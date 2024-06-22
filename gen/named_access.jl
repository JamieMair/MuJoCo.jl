struct XMacroEntry
    type::Union{Symbol,Expr}
    fieldname::Symbol
    numcols::Union{Expr,String,Symbol,Int}
    numrows::Union{Expr,String,Symbol,Int}
end
struct XGroupMacroEntry
    name::Symbol
    identifier::Symbol
    size_identifier::Symbol
    struct_name::Symbol
    collection_name::Symbol
end
struct XGroupAltNamesMacroEntry
    base_identifier::Symbol
    alt_identifier::Symbol
    struct_name::Symbol
    collection_name::Symbol
end

const type_mapping = Dict(
    :int => :Int32,
    :float => :Float32
)

"""
Makes the input all lowercase, apart from the first letter which is uppercase, and then converts it to a symbol.
"""
function standardise_symbol(input)
    Symbol(uppercasefirst(lowercase(string(input))))
end

function parse_x_defs(file)
    xmacros_dict = Dict{Symbol,Dict{Symbol,Vector{XMacroEntry}}}()
    xviewgroups = Dict{Symbol,Dict{Symbol,XGroupMacroEntry}}()
    xviewgroupaltnames = Dict{Symbol,Dict{Symbol,XGroupAltNamesMacroEntry}}()
    open(file, "r") do io
        def_macro_regex = r"#define\s+MJ([a-zA-Z]+)\_([a-zA-Z\_]+)\s+"
        while !eof(io)
            line = readline(io)
            m = match(def_macro_regex, line)
            if !isnothing(m)
                struct_name = standardise_symbol(m.captures[begin]) # e.g. Model, Data etc
                if m.captures[end] == "VIEW_GROUPS"
                    xgroupmacros = parse_xgroup_macro!(io)
                    xviewgroups[struct_name] = xgroupmacros
                elseif m.captures[end] == "VIEW_GROUPS_ALTNAMES"
                    xgroupaltnamemacros = parse_xgroup_altnames_macro!(io)
                    xviewgroupaltnames[struct_name] = xgroupaltnamemacros
                else
                    collection_name = standardise_symbol(m.captures[end]) # e.g. Actuator, Body etc

                    xmacros = parse_xmacro!(io)
                    if !haskey(xmacros_dict, struct_name)
                        xmacros_dict[struct_name] = Dict{Symbol,Vector{XMacroEntry}}()
                    end

                    xmacros_dict[struct_name][collection_name] = xmacros
                end
            end
        end
    end

    return xmacros_dict, xviewgroups, xviewgroupaltnames
end

function parse_xmacro!(io)
    entries = XMacroEntry[]
    while true
        info = parse_xmacro_entry!(io)
        if isnothing(info)
            break
        end

        fieldname = Symbol(string(info.prefix) * string(info.suffix))
        entry = XMacroEntry(info.type, fieldname, info.numcols, info.numrows)
        push!(entries, entry)
        if info.islast
            break
        end
    end
    return entries
end
function parse_xgroup_macro!(io)
    entries = Dict{Symbol,XGroupMacroEntry}()
    while true
        info = parse_xmacro_group_entry!(io)
        if isnothing(info)
            break
        end

        islast, xgroupmacro = info
        entries[xgroupmacro.identifier] = xgroupmacro
        if islast
            break
        end
    end
    return entries
end
function parse_xgroup_altnames_macro!(io)
    entries = Dict{Symbol,XGroupAltNamesMacroEntry}()
    while true
        info = parse_xmacro_group_altnames_entry!(io)
        if isnothing(info)
            break
        end

        islast, xgroupaltnamesmacro = info
        entries[xgroupaltnamesmacro.alt_identifier] = xgroupaltnamesmacro
        if islast
            break
        end
    end
    return entries
end

function simplify_dimension_expr(expr::Any)
    return expr
end
function simplify_dimension_expr(expr::Expr)
    if expr.head == :call
        if expr.args[1] == :MJ_M
            return Expr(:., :model, QuoteNode(expr.args[2]))
        else
            return Expr(:call, simplify_dimension_expr.(expr.args)...)
        end
    else
        return expr
    end
end

function _process_line(line)
    islast = !endswith(line, "\\")
    if !islast
        line = line[begin:end-1] # remove trailing dash
    end
    # replace any empty args
    line = replace(line, r",[^\S\r\n]*," => ",\"\",")
    expr = Meta.parse(line)
    return islast, expr
end

function parse_xentry(line)
    islast, expr = _process_line(line)

    @assert expr.head == :call
    @assert length(expr.args) == 6

    type = expr.args[2] # Type of the X macro
    prefix = expr.args[3]
    suffix = expr.args[4]
    numcols = simplify_dimension_expr(expr.args[5])
    numrows = simplify_dimension_expr(expr.args[6])

    if haskey(type_mapping, type)
        type = type_mapping[type]
    elseif startswith(string(type), "mj")
        type = Expr(:., :LibMuJoCo, QuoteNode(type))
    end

    return (; islast, type, prefix, suffix, numcols, numrows)
end
function parse_xgroup_entry(line)
    islast, expr = _process_line(line)

    @assert expr.head == :call
    @assert length(expr.args) == 5

    # Convert call to struct name
    expr.args[1] = nameof(XGroupMacroEntry)
    # Convert other args to symbols
    expr.args[2:end-1] .= QuoteNode.(expr.args[2:end-1])
    # Convert last argument into same format as before
    (mjtype, collectionname) = split(string(expr.args[end]), "_")
    struct_name = standardise_symbol(mjtype[3:end]) # remove mj
    collectionname = standardise_symbol(collectionname)

    expr.args[end] = QuoteNode(struct_name)
    push!(expr.args, QuoteNode(collectionname))

    return islast, Base.eval(Main, expr)
end
function parse_xgroup_altnames_entry(line)
    islast, expr = _process_line(line)

    @assert expr.head == :call
    @assert length(expr.args) == 4

    # Convert call to struct name
    expr.args[1] = nameof(XGroupAltNamesMacroEntry)
    # Convert other args to symbols
    expr.args[2:end-1] .= QuoteNode.(expr.args[2:end-1])
    # Convert last argument into same format as before
    (mjtype, collectionname) = split(string(expr.args[end]), "_")
    struct_name = standardise_symbol(mjtype[3:end]) # remove mj
    collectionname = standardise_symbol(collectionname)

    expr.args[end] = QuoteNode(struct_name)
    push!(expr.args, QuoteNode(collectionname))

    return islast, Base.eval(Main, expr)
end

function parse_xmacro_entry!(io)
    line = strip(readline(io))
    if line == ""
        return nothing
    end

    if startswith(line, "XGROUP")
        return error("Unexpected macro.")
    elseif startswith(line, "X")
        return parse_xentry(line)
    end
end
function parse_xmacro_group_entry!(io)
    line = strip(readline(io))
    if line == ""
        return nothing
    end

    if !startswith(line, "XGROUP")
        return error("Unexpected macro.")
    else
        return parse_xgroup_entry(line)
    end
end
function parse_xmacro_group_altnames_entry!(io)
    line = strip(readline(io))
    if line == ""
        return nothing
    end

    if !startswith(line, "XGROUP")
        return error("Unexpected macro.")
    else
        return parse_xgroup_altnames_entry(line)
    end
end

function collection_struct_name(original_struct_name, collection_name)
    Symbol(string(original_struct_name) * string(collection_name))
end



function pretty_print_show_fns(type_expr, description)
    fn_expr_long = :(function Base.show(io::IO, ::MIME"text/plain", x::$(type_expr))
        println(io, $(description))
        max_spaces = mapreduce(x -> length(string(x)), max, propertynames(x))
        for pname in propertynames(x)
            prop = getproperty(x, pname)
            print(io, pname)
            print(io, ":")
            num_spaces = 2 + (max_spaces - length(string(pname)))
            print(io, " "^num_spaces)
            if typeof(prop) <: AbstractArray
                show_array(io, prop)
            else
                show(io, prop)
            end
            println(io, "")
        end
    end)
    fn_expr_short = :(function Base.show(io::IO, x::$(type_expr))
        print(io, $(description))
        if hasproperty(x, :name)
            print(io, " \"")
            print(io, getproperty(x, :name))
            print(io, "\"")
        else
            print(io, " Unnamed")
        end
        nothing
    end)
    return fn_expr_short, fn_expr_long
end

function new_propname(fieldname)
    fieldname_str = string(fieldname)

    first_underscore_idx = findfirst('_', fieldname_str)
    if isnothing(first_underscore_idx)
        # If nothing found, use original name
        return fieldname
    end

    return Symbol(fieldname_str[first_underscore_idx+1:end])
end
ptr_inner_type(::Type{Ptr{T}}) where {T} = T

const obj_identifier_to_mjOBJ_map = Dict{Symbol,Expr}(
    :light => :(LibMuJoCo.mjOBJ_LIGHT),
    :cam => :(LibMuJoCo.mjOBJ_CAMERA),
    :camera => :(LibMuJoCo.mjOBJ_CAMERA),
    :actuator => :(LibMuJoCo.mjOBJ_ACTUATOR),
    :body => :(LibMuJoCo.mjOBJ_BODY),
    :geom => :(LibMuJoCo.mjOBJ_GEOM),
    :jnt => :(LibMuJoCo.mjOBJ_JOINT),
    :joint => :(LibMuJoCo.mjOBJ_JOINT),
    :sensor => :(LibMuJoCo.mjOBJ_SENSOR),
    :site => :(LibMuJoCo.mjOBJ_SITE),
    :tendon => :(LibMuJoCo.mjOBJ_TENDON),
    :ten => :(LibMuJoCo.mjOBJ_TENDON),
    :eq => :(LibMuJoCo.mjOBJ_EQUALITY),
    :equality => :(LibMuJoCo.mjOBJ_EQUALITY),
    :key => :(LibMuJoCo.mjOBJ_KEY),
    :keyframe => :(LibMuJoCo.mjOBJ_KEY),
    :numeric => :(LibMuJoCo.mjOBJ_NUMERIC),
    :mat => :(LibMuJoCo.mjOBJ_MATERIAL),
    :material => :(LibMuJoCo.mjOBJ_MATERIAL),
    :texture => :(LibMuJoCo.mjOBJ_TEXTURE),
    :pair => :(LibMuJoCo.mjOBJ_PAIR),
    :hfield => :(LibMuJoCo.mjOBJ_HFIELD),
    :tuple => :(LibMuJoCo.mjOBJ_TUPLE),
    :skin => :(LibMuJoCo.mjOBJ_SKIN),
    :excludes => :(LibMuJoCo.mjOBJ_EXCLUDE),
    :tex => :(LibMuJoCo.mjOBJ_TEXTURE)
)

function doc_fn(docs, fn_expr)
    return Expr(:macrocall, :(Core.var"@doc"), LineNumberNode(1), docs, fn_expr)
end

function construct_plural(name)
    if endswith(name, "y")
        return name[begin:end-1] * "ies"
    else
        return name * "s"
    end
end

function named_access_wrappers_expr(index_xmacro_header_file_path)
    xmacros, xviewgroups, xviewgroupaltnames = parse_x_defs(index_xmacro_header_file_path)

    available_classes = Dict(
        :Data => Wrappers.Data,
        :Model => Wrappers.Model,
    )
    module_name = :LibMuJoCo

    # Get sample filepath to humanoid
    root_path = abspath(joinpath(MuJoCo_jll.artifact_dir, "share", "mujoco", "model", "humanoid"))
    sample_model_path = joinpath(root_path, "humanoid.xml")

    test_model = Wrappers.Model(LibMuJoCo.mj_loadXML(sample_model_path, Ptr{Cvoid}(), "", 0))
    if test_model.internal_pointer == C_NULL
        @error "The file path for the model, or the file itself, could not be loaded: $(sample_model_path)"
    end
    test_data = Wrappers.Data(Wrappers.LibMuJoCo.mj_makeData(test_model), test_model)

    test_classes = Dict(
        :Data => test_data,
        :Model => test_model
    )

    exprs = Expr[]
    exports = Symbol[]

    documented_fns = Set{Symbol}()

    for (struct_name, class_def) in available_classes
        lower_struct_name = Symbol(lowercase(string(struct_name)))
        has_model = hasfield(class_def, :model) && fieldtype(class_def, :model) == available_classes[:Model]

        for collection_name in keys(xmacros[struct_name])
            # Create a struct
            cstruct_name = collection_struct_name(struct_name, collection_name)
            struct_expr = :(
                struct $cstruct_name
                    $(lower_struct_name)::$(struct_name)
                    index::Int
                end
            )
            push!(exprs, struct_expr)
        end

        for (identifier, xgroup) in xviewgroups[struct_name]
            push!(exports, identifier)
            collection_name = xgroup.collection_name
            mjobj_expr = if haskey(obj_identifier_to_mjOBJ_map, identifier)
                obj_identifier_to_mjOBJ_map[identifier]
            else
                test_mjobj_sym = Symbol("mjOBJ_$(uppercase(string(identifier)))")
                @info "Inferring that $identifier maps to $test_mjobj_sym"
                try
                    getfield(LibMuJoCo, test_mjobj_sym)
                catch
                    error("Cannot find $test_mjobj_sym in LibMuJoCo, trying for identifier $identifier.")
                end
                :(LibMuJoCo.$(test_mjobj_sym))
            end

            cstruct_name = collection_struct_name(struct_name, collection_name)

            fn_block_exprs = Expr[]
            push!(fn_block_exprs, :($(lower_struct_name) = getfield(x, $(QuoteNode(lower_struct_name)))))
            if has_model
                push!(fn_block_exprs, :(model = getfield($(lower_struct_name), :model)))
            end
            push!(fn_block_exprs, :(index = getfield(x, :index)))

            property_names = Symbol[:id, :name]

            for xmacro in xmacros[struct_name][collection_name]
                propname = new_propname(xmacro.fieldname)

                if xmacro.numcols == xgroup.size_identifier
                    # return a view into the array
                    test_item = test_classes[struct_name]
                    inferred_return_type = typeof(getproperty(test_item, xmacro.fieldname))
                    if inferred_return_type <: Union{Nothing, AbstractArray}
                        push!(property_names, propname)
                        get_array_expr = :($(lower_struct_name).$(xmacro.fieldname))
                        second_dims = if xmacro.numrows isa Symbol && startswith(string(xmacro.numrows), "mj")
                            :(Base.OneTo($(module_name).$(xmacro.numrows)))
                        elseif xmacro.numrows == 1
                            :(Base.OneTo(1))
                        else
                            Expr(:call, :(Base.OneTo), xmacro.numrows)
                        end
                        return_expr = Expr(:return, Expr(:if, Expr(:call, :isnothing, get_array_expr), nothing, Expr(:call, :view, get_array_expr, :(index + 1), second_dims)))
                        entry_expr = :(f === $(QuoteNode(propname)) && $(return_expr))
                        push!(fn_block_exprs, entry_expr)
                    elseif inferred_return_type <: Ptr
                        push!(property_names, propname)
                        num_elements = if xmacro.numrows isa Symbol && startswith(string(xmacro.numrows), "mj")
                            :($(module_name).$(xmacro.numrows))
                        else
                            xmacro.numrows
                        end
                        element_type = ptr_inner_type(inferred_return_type)
                        return_expr = Expr(:return, quote
                            size_arr = Int($(num_elements))
                            offset = index * size_arr * sizeof($element_type)
                            arr_pointer = Ptr{$element_type}($(lower_struct_name).$(xmacro.fieldname) + offset)
                            if arr_pointer == C_NULL
                                nothing
                            else
                                UnsafeArray(arr_pointer, (size_arr,))
                            end
                        end)
                        entry_expr = :(f === $(QuoteNode(propname)) && $(return_expr))
                        push!(fn_block_exprs, entry_expr)
                    else
                        @info "Property $(xmacro.fieldname) of $class_def is not an array, cannot wrap."
                    end
                end
            end

            # Add function stub with some documentation
            if !(identifier in documented_fns)
                empty_fn_expr = :(function $(identifier) end)
                identifier_docs = "\t$(identifier)([model, data], [name, index])\nCreates an object with access to views of the supplied model or data object, based either on an index or a name. Index refers to MuJoCo IDs, which start at 0. Properties available are:\n$(Expr(:tuple, property_names...))"
                push!(exprs, doc_fn(identifier_docs, empty_fn_expr))
                push!(documented_fns, identifier)
            end

            # Create functions to create the struct objects (both original and "Named" versions)
            push!(exprs, :(function $(identifier)($(lower_struct_name)::$(struct_name), index::Integer)
                return $(cstruct_name)($(lower_struct_name), index)
            end))
            push!(exprs, :(function $(identifier)($(lower_struct_name)::$(struct_name), name::String)
                index = index_by_name($(lower_struct_name), $(mjobj_expr), name)
                return $(cstruct_name)($(lower_struct_name), index)
            end))

            named_type = Symbol("Named" * string(struct_name))
            push!(exprs, :(function $(identifier)($(lower_struct_name)::$(named_type), index::Integer)
                return $(cstruct_name)(getfield($(lower_struct_name), $(QuoteNode(lower_struct_name))), index)
            end))
            push!(exprs, :(function $(identifier)($(lower_struct_name)::$(named_type), name::Symbol)
                index = index_by_name($(lower_struct_name), $(QuoteNode(identifier)), name)
                return $(cstruct_name)(getfield($(lower_struct_name), $(QuoteNode(lower_struct_name))), index)
            end))

            pp_short_fn, pp_long_fn = pretty_print_show_fns(cstruct_name, "$cstruct_name:")

            push!(exprs, pp_short_fn)
            push!(exprs, pp_long_fn)

            # Finish creating the getproperty function

            push!(fn_block_exprs, :(f === :id && return index))
            push!(fn_block_exprs, :(f === :name && return name_by_index(model, $(mjobj_expr), index)))

            push!(exprs, Expr(:function, :(Base.propertynames(::$(cstruct_name))), Expr(:block, Expr(:tuple, QuoteNode.(property_names)...))))

            push!(fn_block_exprs, :(error("Could not find the property: " * string(f))))

            fn_sig = :(Base.getproperty(x::$(cstruct_name), f::Symbol))
            push!(exprs, Expr(:function, fn_sig, Expr(:block, fn_block_exprs...)))
        end

        # Create plural getters
        for (identifier, xgroup) in xviewgroups[struct_name]
            lower_struct_name = Symbol(lowercase(string(struct_name)))
            plural = Symbol(construct_plural(string(identifier)))

            test_class = test_classes[struct_name]
            entry_expr = if hasproperty(test_class, xgroup.size_identifier)
                :(nentries = getproperty($lower_struct_name, $(QuoteNode(xgroup.size_identifier))))
            elseif has_model && hasproperty(test_model, xgroup.size_identifier)
                :(nentries = getproperty(getfield($(lower_struct_name), :model), $(QuoteNode(xgroup.size_identifier))))
            else
                @error "Cannot find $(xgroup.size_identifier) in the $(class_def) struct or in a struct which can be accessed."
                nothing
            end

            return_expr = :(return Tuple($(identifier)($lower_struct_name, i) for i in 0:(nentries-1)))
            fn_sig = :($(plural)($(lower_struct_name)::$(struct_name)))
            fn_expr = Expr(:function, fn_sig, Expr(:block, entry_expr, return_expr))
            push!(exprs, fn_expr)
            push!(exports, plural)
        end
    end


    for (struct_name, xaltname_dict) in xviewgroupaltnames
        if !haskey(available_classes, struct_name)
            @info "Skipping alternative view names for named access for struct $struct_name"
            continue
        end

        for (alt_identifier, xaltname) in xaltname_dict
            alt_plural = Symbol(construct_plural(string(alt_identifier)))
            base_plural = Symbol(construct_plural(string(xaltname.base_identifier)))

            push!(exports, alt_identifier)
            push!(exports, alt_plural)
            push!(exprs, :(const $(alt_identifier) = $(xaltname.base_identifier)))
            push!(exprs, :(const $(alt_plural) = $(base_plural)))
        end
    end

    name_fn = :(function name(x)
        if hasproperty(x, :name) || hasfield(x, :name)
            return x.name
        else
            @error "Tried to get the name of a $(typeof(x)), but no name field or property exists."
        end
    end)
    push!(exprs, name_fn)
    push!(exports, :name)

    export_expr = Expr(:export, unique(exports)...)

    module_expr = Expr(:module, true, :NamedAccess, Expr(:block,
        :(import ..LibMuJoCo),
        :(import ..Utils: show_array),
        :(using UnsafeArrays),
        :(import ..Data),
        :(import ..Model),
        :(include("named_model.jl")),
        export_expr,
        exprs...
    ))
    return module_expr
end

function generate_named_access()
    filepath = joinpath(abspath("."), "mujoco", "python", "mujoco", "indexer_xmacro.h")
    expr = named_access_wrappers_expr(filepath)
    dest_filepath = joinpath(staging_dir, "named_access.jl")
    create_file_from_expr(dest_filepath, expr)
end