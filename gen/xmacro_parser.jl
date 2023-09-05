struct XMacroEntry
    type::Union{Symbol, Expr}
    fieldname::Symbol
    numcols::Union{Expr, String, Symbol, Int}
    numrows::Union{Expr, String, Symbol, Int}
end
struct XMacro
    structname::Symbol
    itemname::Symbol
    entries::Vector{XMacroEntry}
end

const type_mapping = Dict(
    :int => :Int32,
    :float => :Float32
)

function parse_xmacros(file)
    xmacros = XMacro[]
    open(file, "r") do io
        xmacro_start_regex = r"#define\s+MJ([a-zA-Z]+)\_([a-zA-Z]+)\s+"
        while !eof(io)
            line = readline(io)
            m = match(xmacro_start_regex, line)
            if !isnothing(m)
                structname = Symbol(uppercasefirst(lowercase(m.captures[begin]))) # e.g. Model, Data etc
                itemname = Symbol(uppercasefirst(lowercase(m.captures[end]))) # e.g. Actuator, Body etc

                xmacro = parse_xmacro!(io, structname, itemname)
                push!(xmacros, xmacro)
            end
        end
    end
    return xmacros
end

function parse_xmacro!(io, structname, itemname)
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
    return XMacro(structname, itemname, entries)
end

function simplify_dimension_expr(expr::Any)
    return expr
end
function simplify_dimension_expr(expr::Expr)
    if expr.head == :call
        if expr.args[1] == :MJ_M
            return Expr(:., :model, QuoteNode(expr.args[2]))
        else
            return Expr(:call, simplify_dimension_expr.(expr.args))
        end
    else
        return expr
    end
end

function parse_xentry(line)
    islast = !endswith(line, "\\")
    if !islast
        line = line[begin:end-1] # remove trailing dash
    end
    # replace any empty args
    line = replace(line, r",[^\S\r\n]*,"=>",\"\",")
    # convert to an expression
    expr = Meta.parse(line)
    if expr.head != :call
        @show line
        @show expr
    end
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

function parse_xmacro_entry!(io)
    line = strip(readline(io))
    if line == ""
        return nothing
    end

    if startswith(line, "XGROUP")
        return nothing
    elseif startswith(line, "X")
        return parse_xentry(line)
    end
end