# A script for generating the API from the existing LibMuJoCo files
include("mjmacro_parsing.jl")
include("LibMuJoCo/LibMuJoCo.jl")
import .LibMuJoCo

function ntuple_to_array_extents(::Type{NTuple{N, T}}) where {N, T}
    return (N, ntuple_to_array_extents(T)...)
end
function ntuple_to_array_extents(::Type{<:Any})
    return () # Return empty tuple
end

function ntuple_type(::Type{NTuple{N, T}}) where {N, T}
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
structinfo(T) = [(fieldoffset(T,i), fieldname(T,i), fieldtype(T,i)) for i = 1:fieldcount(T)];


function generate_getproperty_fn(mj_struct, new_name::Symbol, all_wrappers, match_macroinfo)
    struct_to_new_symbol_mapping = Dict(Base.getglobal(LibMuJoCo, sn)=>s for (sn, s) in all_wrappers)
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
            dims_expr = Expr(:tuple, extents...)
            Expr(:return, Expr(:call, :UnsafeArray, Expr(:call, Expr(:curly, :Ptr, array_type), Expr(:call, :+, :internal_pointer, foffset)), dims_expr))
        elseif ftype <: Ptr && !isnothing(match_macroinfo) && haskey(first(match_macroinfo), fname)
            finfo = first(match_macroinfo)[fname]
            # TODO: Make sure the datatypes match
            converted_array_sizes = map(finfo.array_sizes) do a
                if a isa Symbol
                    return Expr(:call, :Int, Expr(:., :x, QuoteNode(a)))
                else
                    return Expr(:call, :Int, a)
                end
            end
            dims_expr = Expr(:tuple, converted_array_sizes...)
            pointer_to_pointer = Expr(:call, :unsafe_load, Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, foffset)))
            Expr(:return, Expr(:call, :UnsafeArray, pointer_to_pointer, dims_expr))
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
    struct_to_new_symbol_mapping = Dict(Base.getglobal(LibMuJoCo, sn)=>s for (sn, s) in all_wrappers)
    set_property_lines = Expr[]
    offset = 0

    # Add a local variable for the internal pointer
    push!(set_property_lines, Expr(:(=), :internal_pointer, Expr(:call, :getfield, :x, QuoteNode(:internal_pointer))))
    # Allow the struct to reference internal pointer, overriding any internal names
    push!(set_property_lines, Expr(:(&&), Expr(:call, :(===), :f, QuoteNode(:internal_pointer)), :(error("Cannot set the internal pointer, create a new struct instead."))))
    
    pointer_field_symbols = Symbol[]
    array_field_symbols = Symbol[]

    for (fname, ftype) in zip(fieldnames(mj_struct), fieldtypes(mj_struct))
        @assert fname != :internal_pointer "Struct field cannot be accessed as it conflicts with an internal name."
        cmp_expr = Expr(:call, :(===), :f, QuoteNode(fname))

        set_block = if ftype <: Ptr
            push!(pointer_field_symbols, fname)
            continue
        elseif ftype <: NTuple # Array type
            push!(array_field_symbols, fname)
            continue
        else
            ptr_expr = Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, offset))
            convert_expr = Expr(:call, :convert, ftype, :value)
            local_var_expr = Expr(:(=), :cvalue, convert_expr)
            store_expr = Expr(:call, :unsafe_store!, ptr_expr, :cvalue)
            return_expr = Expr(:return, :cvalue)
            Expr(:block, local_var_expr, store_expr, return_expr)
        end
        push!(set_property_lines, Expr(:if, cmp_expr, set_block))
        
        offset += sizeof(ftype)
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


function build_struct_wrapper(struct_name::Symbol, new_name::Symbol, all_wrappers::Dict{Symbol, Symbol}, parsed_macro_info_dict)
    mj_struct = Base.getglobal(LibMuJoCo, struct_name)
    field_info_dict = nothing
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
    set_prop_fn_expr =generate_setproperty_fn(mj_struct, new_name, all_wrappers)

    return (wrapped_struct, fn_expr, propnames_expr, set_prop_fn_expr)
end

begin
    struct_wrappers = Dict{Symbol, Symbol}(
        :mjData => :Data,
        :mjModel => :Model,
        :mjStatistic => :Statistics,
        :mjOption => :Options,
    )
    parsed_macro_info = parse_macro_file(macro_file)
    other_exprs = Expr[]
    first_exprs = Expr[]
    push!(first_exprs, :(using UnsafeArrays))
    push!(first_exprs, Expr(:export, values(struct_wrappers)...))
    
    for (k, v) in struct_wrappers
        ws, fe, pn, spfn = build_struct_wrapper(k, v, struct_wrappers, parsed_macro_info)
        push!(first_exprs, ws)
        push!(other_exprs, pn)
        push!(other_exprs, fe)
        push!(other_exprs, spfn)
    end
    # TODO sort the `first_exprs` array in topological order

    exprs = vcat(first_exprs, other_exprs)

    create_file_from_expr(joinpath(staging_dir, "wrappers.jl"), exprs)
end