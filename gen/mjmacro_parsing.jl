# Assume mujoco has already been downloaded and checked out
mujoco_dir = normpath(joinpath("$(@__DIR__)/mujoco"))

macro_file = joinpath(mujoco_dir, "include", "mujoco", "mjxmacro.h")


function convert_dep_structname(name::Symbol)
    name_str = string(name)
    return Symbol(lowercasefirst(replace(name_str, "mj"=>"")))
end
struct XMacroFieldInfo
    macro_type::Symbol
    datatype::Symbol
    fieldname::Symbol
    array_sizes::Vector{Union{Expr,Int,Symbol}}
end
recursive_replace(deps, call_mapping, expr::Any) = expr # default
function recursive_replace(deps, call_mapping, expr::Expr)
    if expr.head == :call
        fn_name = expr.args[1]
        if haskey(call_mapping, fn_name)
            push!(deps, call_mapping[fn_name])
            if length(expr.args) != 2
                @error "Call to $(call_mapping[fn_name]) should only have a single parameter"
            end
            return Expr(:., convert_dep_structname(call_mapping[fn_name]), QuoteNode(expr.args[2]))
        end
    end
    return Expr(expr.head, recursive_replace.(Ref(deps), Ref(call_mapping), expr.args)...)
end

function injest_pointer_def(io)
    field_def_macro_regex = r"X(MJV)?\s*\(([a-zA-Z0-9\.\(\)\s,\_\*\+\-]+)\)"
    match_call_macros = Dict{Symbol, Symbol}(
        :MJ_D => :mjData,
        :MJ_M => :mjModel
    )

    field_infos = Dict{Symbol, XMacroFieldInfo}()
    deps = Set{Symbol}()
    while true
        line = strip(readline(io))
        m = match(field_def_macro_regex, line)
        if isnothing(m)
            break
        end
        m_type = isnothing(m.captures[begin]) ? (:X) : (:XMJV)
        arguments = split(m.captures[end], ",")
        dtype = Symbol(strip(arguments[1]))
        fname = Symbol(strip(arguments[2]))

        arr_sizes = map(3:length(arguments)) do i
            s = strip(arguments[i])
            expr = recursive_replace(deps, match_call_macros, Meta.parse(s))
            return expr
        end
        if haskey(field_infos, fname)
            @error "Repeated key of $fname found when parsing macros."
        end
        field_infos[fname] = XMacroFieldInfo(m_type, dtype, fname, arr_sizes)
    end
    
    return field_infos, deps
end

function parse_macro_file(macro_file)
    pointer_def_regex = r"#define MJ([a-zA-Z0-9]+)\_(?:[a-zA-Z0-9\_]*)?POINTERS(?:[a-zA-Z0-9\_]*)?"
    size_definitions = Dict{Symbol, Tuple{Dict{Symbol, XMacroFieldInfo}, Set{Symbol}}}()
    open(macro_file, "r") do io
        while !eof(io)
            next_line = readline(io)
            m = match(pointer_def_regex, next_line)
            if !isnothing(m)
                @info "Parsing $next_line"
                struct_name = Symbol("mj" * uppercasefirst(lowercase(m.captures[begin])))
                new_dict, deps = injest_pointer_def(io)
                if haskey(size_definitions, struct_name)
                    existing_dict, existing_deps = size_definitions[struct_name]
                    for (k, v) in new_dict
                        if haskey(existing_dict, k)
                            @error "Found conflicting key $k in $struct_name when processing macros. Line: $next_line"
                        end
                        existing_dict[k] = v
                    end
                    for d in deps
                        push!(existing_deps, d)
                    end
                else
                    size_definitions[struct_name] = (new_dict, deps)
                end
            end
        end
    end

    return size_definitions
end