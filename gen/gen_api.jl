# A script for generating the API from the existing LibMuJoCo files
include("LibMuJoCo/LibMuJoCo.jl")
import .LibMuJoCo

function generate_getproperty_fn(mj_struct, new_name::Symbol)
    get_property_lines = Expr[]
    offset = 0

    # Add a local variable for the internal pointer
    push!(get_property_lines, Expr(:(=), :internal_pointer, Expr(:call, :getfield, :x, QuoteNode(:internal_pointer))))
    # Allow the struct to reference internal pointer, overriding any internal names
    push!(get_property_lines, Expr(:(&&), Expr(:call, :(===), :f, QuoteNode(:internal_pointer)), Expr(:return, :internal_pointer)))
    

    for (fname, ftype) in zip(fieldnames(mj_struct), fieldtypes(mj_struct))
        @assert fname != :internal_pointer "Struct field cannot be accessed as it conflicts with an internal name."
        cmp_expr = Expr(:call, :(===), :f, QuoteNode(fname))
        

        rtn_expr = if ftype <: Ptr
            Expr(:return, Expr(:call, ftype, Expr(:call, :+, :internal_pointer, offset)))
        else
            Expr(:return, Expr(:call, :unsafe_load, Expr(:call, Expr(:curly, :Ptr, ftype), Expr(:call, :+, :internal_pointer, offset))))
        end
        return_expr = Expr(:(&&), cmp_expr, rtn_expr)
        push!(get_property_lines, return_expr)
        offset += sizeof(ftype)
    end

    push!(get_property_lines, :(error("Could not find property $f")))

    # TODO refactor this fn to have better variable names
    fn_block = Expr(:block, get_property_lines...)
    fn_expr = Expr(:function, Expr(:call, :(Base.getproperty), Expr(:(::), :x, new_name), Expr(:(::), :f, :Symbol)), fn_block)


    return fn_expr
end
function generate_propertynames_fn(mj_struct, new_name::Symbol)
    prop_names = Expr(:tuple, QuoteNode.(Symbol.(fieldnames(mj_struct)))...)
    propnames_expr = Expr(:function, Expr(:call, :(Base.propertynames), Expr(:(::), :x, new_name)), Expr(:block, prop_names))
    return propnames_expr
end


function build_struct_wrapper(struct_name::Symbol, new_name::Symbol)
    mj_struct = Base.getglobal(LibMuJoCo, struct_name)

    wrapped_struct = Expr(:struct, false, new_name, Expr(:block, Expr(:(::), :internal_pointer, Expr(:curly, :Ptr, struct_name))))

    fn_expr = generate_getproperty_fns(mj_struct, new_name)
    propnames_expr = generate_propertynames_fn(mj_struct, new_name)

    return (wrapped_struct, fn_expr, propnames_expr)
end

begin
    
    struct_wrappers = Dict{Symbol, Symbol}(
        :mjData => :Data,
        :mjModel => :Model,
    )

    exprs = Expr[]
    for (k, v) in struct_wrappers
        ws, fe, pn = build_struct_wrapper(k, v)
        push!(exprs, ws)
        push!(exprs, fe)
        push!(exprs, pn)
    end

    create_file_from_expr(joinpath(staging_dir, "wrappers.jl"), exprs)
end