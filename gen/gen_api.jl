# A script for generating the API from the existing LibMuJoCo files
include("mjmacro_parsing.jl")
function ntuple_to_array_extents(::Type{NTuple{N,T}}) where {N,T}
    return (N, ntuple_to_array_extents(T)...)
end
function ntuple_to_array_extents(::Type{<:Any})
    return () # Return empty tuple
end

function ntuple_type(::Type{NTuple{N,T}}) where {N,T}
    return ntuple_type(T)
end
function ntuple_type(::Type{T}) where {T}
    return T
end

function inner_type(::Type{Ptr{T}}) where {T}
    return T
end

function try_wrap_pointer(::Type{T}, expr, mapping) where {T}
    if haskey(mapping, T)
        Expr(:call, mapping[T], expr)
    else
        expr
    end
end
function try_wrap_pointer(::Type{Ptr{T}}, expr, mapping) where {T}
    try_wrap_pointer(T, expr, mapping)
end
structinfo(T) = [(fieldoffset(T, i), fieldname(T, i), fieldtype(T, i)) for i = 1:fieldcount(T)];


function gen_auto_cconvert_fn(mj_struct_name, new_name)
    target_type_expr = Expr(:(::), Expr(:curly, :Type, Expr(:curly, :Ptr, mj_struct_name)))
    original_type_expr = Expr(:(::), :wrapper, new_name)
    method_sig = Expr(:call, Expr(:., :Base, QuoteNode(:cconvert)), target_type_expr, original_type_expr)
    method_block = Expr(:block, Expr(:return, Expr(:., :wrapper, QuoteNode(:internal_pointer))))
    return Expr(:function, method_sig, method_block)
end


function generate_getproperty_fn(mj_struct, new_name::Symbol, all_wrappers, match_macroinfo)
    struct_to_new_symbol_mapping = Dict(Base.getglobal(LibMuJoCo, sn) => s for (sn, s) in all_wrappers)
    get_property_lines = Expr[]
    offset = 0

    # Add a local variable for the internal pointer
    push!(get_property_lines, Expr(:(=), :internal_pointer, Expr(:call, :getfield, :x, QuoteNode(:internal_pointer))))

    # Add lines to get a variable to the dependency
    if !isnothing(match_macroinfo)
        _, extra_deps = match_macroinfo
        for d in extra_deps
            conv_name = convert_dep_structname(d)
            if d == Symbol(string(nameof(mj_struct))[begin:end-1]) # remove _ on the end
                push!(get_property_lines, Expr(:(=), conv_name, :x))
            else
                push!(get_property_lines, Expr(:(=), conv_name, Expr(:call, :getfield, :x, QuoteNode(conv_name))))
            end
        end
    end

    # Allow the struct to reference internal pointer, overriding any internal names
    push!(get_property_lines, Expr(:(&&), Expr(:call, :(===), :f, QuoteNode(:internal_pointer)), Expr(:return, :internal_pointer)))



    for (foffset, fname, ftype) in structinfo(mj_struct)
        @assert fname != :internal_pointer "Struct field cannot be accessed as it conflicts with an internal name."
        cmp_expr = Expr(:call, :(===), :f, QuoteNode(fname))

        foffset = Int64(foffset) # convert to Int64 for readability

        rtn_expr = if ftype <: NTuple # Specially wrap array type
            # Get the extents from the type
            array_type = ntuple_type(ftype)
            extents = ntuple_to_array_extents(ftype)
            # TODO: Check whether the wrapped array has row/column major consistency
            # TODO: Check consistency with the struct mapping
            # TODO: Explicitly choose own=false in the `unsafe_wrap call`
            dims_expr = Expr(:tuple, Base.reverse(extents)...)
            pointer_expr = Expr(:call, Expr(:curly, :Ptr, array_type), Expr(:call, :+, :internal_pointer, foffset))
            if length(extents) > 1
                Expr(:return, Expr(:call, :transpose, Expr(:call, :UnsafeArray, pointer_expr, dims_expr)))
            else
                Expr(:return, Expr(:call, :UnsafeArray, pointer_expr, dims_expr))
            end
        elseif ftype <: Ptr && !isnothing(match_macroinfo) && haskey(first(match_macroinfo), fname)
            finfo = first(match_macroinfo)[fname]
            # TODO: Make sure the datatypes match
            converted_array_sizes = map(finfo.array_sizes) do a
                if a isa Symbol
                    # Add workaround
                    if hasfield(mj_struct, a)
                        return Expr(:call, :Int, Expr(:., :x, QuoteNode(a)))
                    else
                        # Try to find the location where this field exists
                        deps = last(match_macroinfo)
                        for d in deps
                            other_mj_struct = Base.getglobal(LibMuJoCo, d)
                            if other_mj_struct == mj_struct
                                continue
                            end
                            if hasfield(other_mj_struct, a)
                                s_name = string(nameof(other_mj_struct))
                                if endswith(s_name, "_")
                                    s_name = s_name[begin:end-1]
                                end
                                @info "Found issue in mujoco's library. Field $a is not available in $(nameof(mj_struct)), but found one in $s_name"
                                return Expr(:call, :Int, Expr(:., convert_dep_structname(Symbol(s_name)), QuoteNode(a)))
                            end
                        end
                        return nothing
                    end
                else
                    return Expr(:call, :Int, a)
                end
            end
            if any(isnothing, converted_array_sizes)
                @info "Could not find the appropriate sizes for field $fname of $(nameof(mj_struct)). Not converting to array."
                ptr_expr = Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, foffset))
                return_value = if haskey(struct_to_new_symbol_mapping, ftype)
                    Expr(:return, try_wrap_pointer(ftype, ptr_expr, struct_to_new_symbol_mapping))
                else
                    Expr(:return, Expr(:call, :unsafe_load, ptr_expr))
                end
                return_value
            else
                # Transpose for row-major order
                dims_expr = Expr(:tuple, Iterators.reverse(converted_array_sizes)...)
                non_integer_array_sizes = filter(converted_array_sizes) do _arr_size_expr
                    if (_arr_size_expr isa Expr)
                        if (_arr_size_expr.head == :call)
                            if (length(_arr_size_expr.args) == 2)
                                if (_arr_size_expr.args[1] == :Int) && (_arr_size_expr.args[2] isa Integer) && _arr_size_expr.args[2] != 0
                                    return false
                                end
                            end
                        end
                    end
                    return true
                end
                # TODO: Fix to return an arbitrary 0 sized array!
                dims_expr = Expr(:tuple, Iterators.reverse(converted_array_sizes)...)

                all_non_zero_expr = Expr(:call, :all, Expr(:call, :!=, 0), Expr(:tuple, non_integer_array_sizes...))

                pointer_to_pointer = Expr(:call, :unsafe_load, Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, foffset)))
                pointer_load = Expr(:(=), :_ptr, pointer_to_pointer)
                arr_return_expr = Expr(:block, pointer_load, Expr(:if, :(_ptr == C_NULL), nothing, Expr(:call, :transpose, Expr(:call, :UnsafeArray, :_ptr, dims_expr))))
                if_expr = Expr(:if, all_non_zero_expr, arr_return_expr, nothing)
                Expr(:return, if_expr)
            end
        else
            ptr_expr = Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, foffset))
            if haskey(struct_to_new_symbol_mapping, ftype)
                Expr(:return, try_wrap_pointer(ftype, ptr_expr, struct_to_new_symbol_mapping))
            else
                Expr(:return, Expr(:call, :unsafe_load, ptr_expr))
            end
        end
        return_expr = Expr(:(&&), cmp_expr, rtn_expr)
        push!(get_property_lines, return_expr)
        offset = max(offset, foffset) + sizeof(ftype)
    end

    expected_struct_size = sizeof(mj_struct)
    if offset != expected_struct_size
        num_padded_bytes = expected_struct_size - offset
        @warn "Padding of $num_padded_bytes bytes in $(nameof(mj_struct)) detected.\nExpected $expected_struct_size bytes, but only accounted for $offset bytes."
    end

    push!(get_property_lines, :(error("Could not find property $f")))

    # TODO refactor this fn to have better variable names
    fn_block = Expr(:block, get_property_lines...)
    fn_expr = Expr(:function, Expr(:call, :(Base.getproperty), Expr(:(::), :x, new_name), Expr(:(::), :f, :Symbol)), fn_block)


    return fn_expr
end

function generate_setproperty_fn(mj_struct, new_name::Symbol, all_wrappers)
    struct_to_new_symbol_mapping = Dict(Base.getglobal(LibMuJoCo, sn) => s for (sn, s) in all_wrappers)
    set_property_lines = Expr[]

    # Add a local variable for the internal pointer
    push!(set_property_lines, Expr(:(=), :internal_pointer, Expr(:call, :getfield, :x, QuoteNode(:internal_pointer))))
    # Allow the struct to reference internal pointer, overriding any internal names
    push!(set_property_lines, Expr(:(&&), Expr(:call, :(===), :f, QuoteNode(:internal_pointer)), :(error("Cannot set the internal pointer, create a new struct instead."))))

    pointer_field_symbols = Symbol[]
    array_field_symbols = Symbol[]

    for (foffset, fname, ftype) in structinfo(mj_struct)
        @assert fname != :internal_pointer "Struct field cannot be accessed as it conflicts with an internal name."
        cmp_expr = Expr(:call, :(===), :f, QuoteNode(fname))

        foffset = Int64(foffset) # convert to Int64 for readability
        
        set_block = if ftype <: Ptr
            push!(pointer_field_symbols, fname)
            continue
        elseif ftype <: NTuple # Array type
            push!(array_field_symbols, fname)
            continue
        else
            ptr_expr = Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, foffset))
            convert_expr = Expr(:call, :convert, ftype, :value)
            local_var_expr = Expr(:(=), :cvalue, convert_expr)
            store_expr = Expr(:call, :unsafe_store!, ptr_expr, :cvalue)
            return_expr = Expr(:return, :cvalue)
            Expr(:block, local_var_expr, store_expr, return_expr)
        end
        push!(set_property_lines, Expr(:if, cmp_expr, set_block))
    end

    if length(array_field_symbols) > 0
        err_expr = Expr(:call, :error, "Cannot overwrite array field. Mutate the array instead.")
        check_expr = Expr(:call, :in, :f, Expr(:tuple, QuoteNode.(array_field_symbols)...))
        push!(set_property_lines, Expr(:if, check_expr, err_expr))
    end
    if length(pointer_field_symbols) > 0
        err_expr = Expr(:call, :error, "Cannot overwrite a pointer field.")
        check_expr = Expr(:call, :in, :f, Expr(:tuple, QuoteNode.(pointer_field_symbols)...))
        push!(set_property_lines, Expr(:if, check_expr, err_expr))
    end

    push!(set_property_lines, :(error("Could not find property $f to set.")))

    fn_block = Expr(:block, set_property_lines...)
    fn_expr = Expr(:function, Expr(:call, :(Base.setproperty!), Expr(:(::), :x, new_name), Expr(:(::), :f, :Symbol), :value), fn_block)


    return fn_expr
end

function generate_propertynames_fn(mj_struct, new_name::Symbol)
    prop_names = Expr(:tuple, QuoteNode.(Symbol.(fieldnames(mj_struct)))...)
    propnames_expr = Expr(:function, Expr(:call, :(Base.propertynames), Expr(:(::), :x, new_name)), Expr(:block, prop_names))
    return propnames_expr
end


function build_struct_wrapper(struct_name::Symbol, new_name::Symbol, all_wrappers::Dict{Symbol,Symbol}, parsed_macro_info_dict)
    mj_struct = Base.getglobal(LibMuJoCo, struct_name)
    extra_deps, match_macroinfo = if !haskey(parsed_macro_info_dict, struct_name)
        (), nothing # empty tuple
    else
        fieldinfo_dict, deps = parsed_macro_info_dict[struct_name]
        filtered_deps = Set{Symbol}()
        for d in deps
            if d != struct_name
                push!(filtered_deps, d)
            end
        end
        other_deps = map(collect(filtered_deps)) do d
            wrapped_type = all_wrappers[d]
            Expr(:(::), convert_dep_structname(d), wrapped_type)
        end
        other_deps, (fieldinfo_dict, deps)
    end

    wrapped_struct = Expr(:struct, false, new_name, Expr(:block, Expr(:(::), :internal_pointer, Expr(:curly, :Ptr, struct_name)), extra_deps...))

    fn_expr = generate_getproperty_fn(mj_struct, new_name, all_wrappers, match_macroinfo)
    propnames_expr = generate_propertynames_fn(mj_struct, new_name)
    set_prop_fn_expr = generate_setproperty_fn(mj_struct, new_name, all_wrappers)
    auto_convert_fn_expr = gen_auto_cconvert_fn(struct_name, new_name)

    return (wrapped_struct, fn_expr, propnames_expr, set_prop_fn_expr, auto_convert_fn_expr)
end

struct MutableStructInfo
    finalizer_expr::Union{Expr,Symbol}
    args::Vector{Union{Symbol,Expr,<:Number,<:AbstractString}}
end
conv_internal_ref(struct_name) = Symbol("__" * lowercase(string(struct_name)))
function insert_regular_gc_ctor!(expr, info::MutableStructInfo)
    @assert expr.head == :struct
    struct_name = expr.args[2]
    # Insert a default constructor to the struct definition
    fields_block = expr.args[end]
    @assert fields_block.head == :block
    # Assume after default constructor
    fields = [a for a in fields_block.args if !(typeof(a) <: LineNumberNode) && a.head == :(::)]
    fields_no_types = [a.args[1] for a in fields]
    instantiate_expr = Expr(:(=), conv_internal_ref(struct_name), Expr(:call, :new, fields_no_types...))
    custom_finalizer = Expr(:call, info.finalizer_expr, info.args...)
    gc_fun_expr = Expr(:function, Expr(:call, :__finalizer, conv_internal_ref(struct_name)), Expr(:block, custom_finalizer))
    finalizer_expr = Expr(:call, :(Base.finalizer), :__finalizer, conv_internal_ref(struct_name))
    return_expr = Expr(:return, conv_internal_ref(struct_name))
    new_ctor_fn = Expr(:function, Expr(:call, struct_name, fields...), Expr(:block, instantiate_expr, gc_fun_expr, finalizer_expr, return_expr))
    push!(fields_block.args, new_ctor_fn)
    return expr
end

function generate_struct_field_docs_helpers(mj_name, wrapped_name)
    exprs = Expr[]
    fields = struct_mapping[string(mj_name)].fields
    fn_sig = Expr(:call, :show_docs, Expr(:(::), Expr(:curly, :Type, wrapped_name)), :(property_name::Symbol))
    fn_body = Expr[]
    for field in fields
        doc_str = replace("$(string(wrapped_name)).$(field.name): $(field.doc)", r"\s+"=>" ")
        field_expr = Expr(:(&&), Expr(:call, :(===), :property_name, QuoteNode(Symbol(field.name))), Expr(:return, Expr(:call, :println, doc_str)))
        push!(fn_body, field_expr)
    end
    push!(fn_body, Expr(:call, :throw, Expr(:call, :ArgumentError, Expr(:string, "The property ", :property_name, " is not defined for $wrapped_name ($mj_name)."))))
    fn = Expr(:function, fn_sig, Expr(:block, fn_body...))
    push!(exprs, fn)
    alt_signature = Expr(:call, :show_docs, Expr(:(::), :x, wrapped_name), :(property_name::Symbol))
    alt_fn = Expr(:function, alt_signature, Expr(:block, Expr(:return, Expr(:call, :show_docs, :(typeof(x)), :property_name))))
    push!(exprs, alt_fn)

    return exprs
end

function create_basic_wrappers()
    parsed_macro_info = parse_macro_file(macro_file)

    struct_wrappers = Dict{Symbol,Symbol}(
        :mjData => :Data,
        :mjModel => :Model,
        :mjStatistic => :Statistics,
        :mjOption => :Options,
    )
    other_exprs = Expr[]
    first_exprs = Expr[]
    push!(first_exprs, :(using UnsafeArrays))
    push!(first_exprs, Expr(:export, values(struct_wrappers)...))

    for (k, v) in struct_wrappers
        ws, fe, pn, spfn, ac = build_struct_wrapper(k, v, struct_wrappers, parsed_macro_info)
        push!(first_exprs, ws)
        push!(other_exprs, pn)
        push!(other_exprs, fe)
        push!(other_exprs, spfn)
        push!(other_exprs, ac)

        # Documentation functions
        doc_fns = generate_struct_field_docs_helpers(k, v)
        for doc_fn in doc_fns
            push!(other_exprs, doc_fn)
        end
    end

    mutable_struct_info = Dict{Symbol,MutableStructInfo}(
        :Data => MutableStructInfo(:mj_deleteData, [Expr(:., conv_internal_ref(:Data), QuoteNode(:internal_pointer))]),
        :Model => MutableStructInfo(:mj_deleteModel, [Expr(:., conv_internal_ref(:Model), QuoteNode(:internal_pointer))])
    )
    mutable_structs = Set(collect(keys(mutable_struct_info))) # Make main structs mutable so they can be garbage collected
    for expr in first_exprs
        if length(expr.args) >= 2
            struct_name = expr.args[2]
            if expr.head == :struct && haskey(mutable_struct_info, struct_name)
                expr.args[1] = true # change to mutable
                # Add in a new constructor
                insert_regular_gc_ctor!(expr, mutable_struct_info[struct_name])
            end
        end
    end

    # TODO sort the `first_exprs` array in topological order

    exprs = vcat(first_exprs, other_exprs)

    create_file_from_expr(joinpath(staging_dir, "wrappers.jl"), exprs)


    struct_wrappers = Dict{Symbol,Symbol}(
        :mjvScene => :VisualiserScene,
        :mjvCamera => :VisualiserCamera,
        :mjvOption => :VisualiserOption,
        :mjrContext => :RendererContext,
        :mjvFigure => :VisualiserFigure,
        :mjvPerturb => :VisualiserPerturb,
    )
    other_exprs = Expr[]
    first_exprs = Expr[]
    push!(first_exprs, :(using UnsafeArrays))
    push!(first_exprs, Expr(:export, values(struct_wrappers)...))

    for (k, v) in struct_wrappers
        ws, fe, pn, spfn, ac = build_struct_wrapper(k, v, struct_wrappers, parsed_macro_info)
        push!(first_exprs, ws)
        push!(other_exprs, pn)
        push!(other_exprs, fe)
        push!(other_exprs, spfn)
        push!(other_exprs, ac)
    end

    mutable_struct_info = Dict{Symbol,MutableStructInfo}(
        :VisualiserScene => MutableStructInfo(:mjv_freeScene, [Expr(:., conv_internal_ref(:VisualiserScene), QuoteNode(:internal_pointer))]),
        :RendererContext => MutableStructInfo(:mjr_freeContext, [Expr(:., conv_internal_ref(:RendererContext), QuoteNode(:internal_pointer))]),
        :VisualiserCamera => MutableStructInfo(Expr(:., :Libc, QuoteNode(:free)), [Expr(:., conv_internal_ref(:VisualiserCamera), QuoteNode(:internal_pointer))]),
        :VisualiserOption => MutableStructInfo(Expr(:., :Libc, QuoteNode(:free)), [Expr(:., conv_internal_ref(:VisualiserOption), QuoteNode(:internal_pointer))]),
        :VisualiserFigure => MutableStructInfo(Expr(:., :Libc, QuoteNode(:free)), [Expr(:., conv_internal_ref(:VisualiserFigure), QuoteNode(:internal_pointer))]),
        :VisualiserPerturb => MutableStructInfo(Expr(:., :Libc, QuoteNode(:free)), [Expr(:., conv_internal_ref(:VisualiserPerturb), QuoteNode(:internal_pointer))])
    )
    mutable_structs = Set(collect(keys(mutable_struct_info))) # Make main structs mutable so they can be garbage collected
    for expr in first_exprs
        if length(expr.args) >= 2
            struct_name = expr.args[2]
            if expr.head == :struct && haskey(mutable_struct_info, struct_name)
                expr.args[1] = true # change to mutable
                # Add in a new constructor
                insert_regular_gc_ctor!(expr, mutable_struct_info[struct_name])
            end
        end
    end

    # TODO sort the `first_exprs` array in topological order

    exprs = vcat(first_exprs, other_exprs)

    create_file_from_expr(joinpath(staging_dir, "visualiser_wrappers.jl"), exprs)
end