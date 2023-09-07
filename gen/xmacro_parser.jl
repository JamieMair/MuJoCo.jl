struct XMacroEntry
    type::Union{Symbol, Expr}
    fieldname::Symbol
    numcols::Union{Expr, String, Symbol, Int}
    numrows::Union{Expr, String, Symbol, Int}
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
    xmacros_dict = Dict{Symbol, Dict{Symbol, Vector{XMacroEntry}}}()
    xviewgroups = Dict{Symbol, Dict{Symbol, XGroupMacroEntry}}()
    xviewgroupaltnames = Dict{Symbol, Dict{Symbol, XGroupAltNamesMacroEntry}}()
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
                        xmacros_dict[struct_name] = Dict{Symbol, Vector{XMacroEntry}}()
                    end
                    
                    xmacros_dict[struct_name][collection_name] = xmacros
                end
            end
        end
    end

    for (struct_name, xaltname_dict) in xviewgroupaltnames
        for (alt_identifier, xaltname) in xaltname_dict
            xviewgroup = xviewgroups[struct_name]
            base_xgroup = xviewgroup[xaltname.base_identifier]

            altname_xgroup = XGroupMacroEntry(base_xgroup.name, alt_identifier, base_xgroup.size_identifier, struct_name, base_xgroup.collection_name)
            xviewgroup[alt_identifier] = altname_xgroup
        end
    end

    return xmacros_dict, xviewgroups
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
    entries = Dict{Symbol, XGroupMacroEntry}()
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
    entries = Dict{Symbol, XGroupAltNamesMacroEntry}()
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
            return Expr(:call, simplify_dimension_expr.(expr.args))
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
    line = replace(line, r",[^\S\r\n]*,"=>",\"\",")
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