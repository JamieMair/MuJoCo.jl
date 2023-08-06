# A script for generating the API from the existing LibMuJoCo files
include("LibMuJoCo/LibMuJoCo.jl")
import .LibMuJoCo


function build_struct_wrapper(struct_name::Symbol, new_name::Symbol)
    mj_struct = Base.getglobal(LibMuJoCo, struct_name)

    wrapped_struct = Expr(:struct, false, new_name, Expr(:block, Expr(:(::), :internal_pointer, Expr(:curly, :Ptr, struct_name))))

    get_property_lines = Expr[]
    offset = 0

    push!(get_property_lines, Expr(:(=), :internal_pointer, Expr(:call, :getfield, :x, QuoteNode(:internal_pointer))))
    prop_names = Expr(:tuple, QuoteNode.(Symbol.(fieldnames(mj_struct)))...)
    for (fname, ftype) in zip(fieldnames(mj_struct), fieldtypes(mj_struct))
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
    fn_block = Expr(:block, get_property_lines...)
    fn_expr = Expr(:function, Expr(:call, :(Base.getproperty), Expr(:(::), :x, new_name), Expr(:(::), :f, :Symbol)), fn_block)

    propnames_expr = Expr(:function, Expr(:call, :(Base.propertynames), Expr(:(::), :x, new_name)), Expr(:block, prop_names))
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