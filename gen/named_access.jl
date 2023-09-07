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

    # for (struct_name, xaltname_dict) in xviewgroupaltnames
    #     for (alt_identifier, xaltname) in xaltname_dict
    #         xviewgroup = xviewgroups[struct_name]
    #         base_xgroup = xviewgroup[xaltname.base_identifier]

    #         altname_xgroup = XGroupMacroEntry(base_xgroup.name, alt_identifier, base_xgroup.size_identifier, struct_name, base_xgroup.collection_name)
    #         xviewgroup[alt_identifier] = altname_xgroup
    #     end
    # end

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

function new_propname(fieldname)
    fieldname_str = string(fieldname)

    first_underscore_idx = findfirst('_', fieldname_str)
    if isnothing(first_underscore_idx)
        # If nothing found, use original name
        return fieldname
    end

    return Symbol(fieldname_str[first_underscore_idx+1:end])
end

function named_access_wrappers_expr(index_xmacro_header_file_path)
    xmacros, xviewgroups, xviewgroupaltnames = parse_x_defs(index_xmacro_header_file_path)
    # TODO add in alt name access
    available_classes = Dict(
        :Data => Wrappers.Data,
        :Model => Wrappers.Model,
    )
    module_name = :LibMuJoCo

    exprs = Expr[]
    exports = Symbol[]

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

            cstruct_name = collection_struct_name(struct_name, collection_name)
            # Create functions to create the struct objects
            push!(exprs, :(function $(identifier)($(lower_struct_name)::$(struct_name), index::Int)
                return $(cstruct_name)($(lower_struct_name), index)
            end))
            push!(exprs, :(function $(identifier)($(lower_struct_name)::$(struct_name), name::Symbol)
                index = index_by_name($(lower_struct_name), name)
                return $(cstruct_name)($(lower_struct_name), index)
            end))


            fn_block_exprs = Expr[]
            push!(fn_block_exprs, :($(lower_struct_name) = x.$(lower_struct_name)))
            if has_model
                push!(fn_block_exprs, :(model = getfield($(lower_struct_name), :model)))
            end
            push!(fn_block_exprs, :(index = x.index))

            property_names = Symbol[]

            for xmacro in xmacros[struct_name][collection_name]
                propname = new_propname(xmacro.fieldname)
                
                if xmacro.numcols == xgroup.size_identifier
                    push!(property_names, propname)
                    # return a view into the array
                    get_array_expr = :($(lower_struct_name).$(xmacro.fieldname))
                    second_dims = if xmacro.numrows isa Symbol && startswith(string(xmacro.numrows), "mj")
                        :(Base.OneTo($(module_name).$(xmacro.numrows)))
                    elseif xmacro.numrows == 1
                        :(Base.OneTo(1))
                    else
                        Expr(:call, :(Base.OneTo), xmacro.numrows)
                    end
                    return_expr = Expr(:return, Expr(:call, :view, get_array_expr, :index, second_dims))
                    entry_expr = :(f == $(QuoteNode(propname)) && $(return_expr))
                    push!(fn_block_exprs, entry_expr)
                end
            end

            push!(exprs, Expr(:function, :(Base.propertynames(::$(cstruct_name))), Expr(:block, Expr(:tuple, QuoteNode.(property_names)...))))

            push!(fn_block_exprs, :(error("Could not find the property: " * string(f))))

            fn_sig = :(Base.getproperty(x::$(cstruct_name), f::Symbol))
            push!(exprs, Expr(:function, fn_sig, Expr(:block, fn_block_exprs...)))
        end


    end

    export_expr = Expr(:export, exports...)

    module_expr = Expr(:module, true, :NamedAccess, Expr(:block, 
        :(import ..LibMuJoCo),
        :(import ..Data),
        :(import ..Model),
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