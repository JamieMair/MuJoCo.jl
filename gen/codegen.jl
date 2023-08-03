import Graphs


const value_type_mapping = Dict{String, Symbol}(
    "char"=>:Cchar,
    "int"=>:Cint,
    "long"=>:Int64,
    "float"=>:Float32,
    "double"=>:Float64,
    "void"=>:Cvoid,
    "unsigned int"=>:Cuint,
    "uintptr_t"=>:Csize_t,
    "size_t"=>:Csize_t
)

parse_type(f::ValueType) = f.name
parse_type(f::PointerType) = parse_type(f.inner_type)
parse_type(f::ArrayType) = parse_type(f.inner_type)
parse_type(f::StructFieldDecl) = parse_type(f.type)
parse_type(f) = nothing


function sort_structs(structs::AbstractArray{StructDecl})
    graph = Graphs.DiGraph(length(structs), 0)
    struct_map = Dict(
        (s.name=>i for (i, s) in enumerate(structs))...
    )
    skipped_lookups = Set{String}()
    for (i, s) in enumerate(structs)
        for f in s.fields
            p = parse_type(f)
            if !isnothing(p) && !haskey(value_type_mapping, p)
                # Name of struct
                if haskey(struct_map, p)
                    j = struct_map[p]
                    Graphs.add_edge!(graph, i, j)
                else
                    push!(skipped_lookups, p)
                end
            end
        end
    end

    for skipped in skipped_lookups
        @info "Skipped $skipped in struct lookup. Must be defined elsewhere."
    end

    sorted_ids = Graphs.topological_sort(graph)
    
    return reverse(structs[sorted_ids])
end

const anon_struct_count = Ref(0)
function replace_anon!(new_structs, item::PointerType)
    return PointerType(replace_anon(item.inner_type), item.is_const, item.is_volatile, item.is_restrict)
end
function replace_anon!(new_structs, item::ArrayType)
    return ArrayType(replace_anon(item.inner_type), item.extents)
end
function replace_anon!(new_structs, item::AnonymousStructDecl)
    new_count = anon_struct_count[] + 1
    anon_struct_count[] = new_count
    new_struct_name = "AnonymousStruct$new_count"
    new_fields = map(item.fields) do f
        replace_anon!(new_structs, f)
    end
    new_struct = StructDecl(new_struct_name, "struct $(new_struct_name)", new_fields)
    push!(new_structs, new_struct)
    return new_struct
end
function replace_anon!(new_structs, item::AnonymousUnionDecl)
    new_count = anon_struct_count[] + 1
    anon_struct_count[] = new_count
    new_struct_name = "AnonymousUnionStruct$new_count"
    field_names = map(item.fields) do f
        if !(typeof(f) <: StructFieldDecl)
            error("Unsupported union structure.")
        end

        f.name
    end
    field_types = map(item.fields) do f
        if !(typeof(f) <: StructFieldDecl)
            error("Unsupported union structure.")
        end

        f.type
    end
    new_union_type= UnionType(new_struct_name, field_names, field_types)
    new_fields = map(item.fields) do f
        replace_anon!(new_structs, f)
    end
    new_struct = StructDecl(new_struct_name, "struct $(new_struct_name)", new_fields)
    push!(new_structs, new_struct)
    return new_struct
end

function fix_anonymous_types(structs::AbstractArray{StructDecl})
    structs_with_unions = 0
    additional_structs = StructDecl[]
    map(structs) do s
        prev_struct_with_unions = structs_with_unions
        if any(typeof(f) isa CAnonymousType for f in s.fields)
            num_unions = 0
            new_fields = map(s.fields) do f
                if typeof(f) isa AnonymousUnionDecl
                    num_unions += 1
                    structs_with_unions = prev_struct_with_unions + 1
                    AnonymousUnionDeclInfo(f, structs_with_unions, num_unions)

                elseif typeof(f) isa AnonymousStructDecl
                    error("Not designed to handle this case.")
                else
                    f
                end
            end
            return StructDecl(s.name, s.declname, new_fields)
        else
            s
        end
    end
end


function declare(enum::EnumDecl)
    pair_values = [(k, v) for (k, v) in enum.values]
    sort!(pair_values; by=(x->x[2]))
    body = Expr(:block, (Expr(:(=), Symbol(k), v) for (k, v) in pair_values)...)
    enum_name = Expr(:(::), Symbol(enum.name), :Cint)
    Expr(:macrocall, Symbol("@cenum"), LineNumberNode(1), enum_name, body)
end

function equivalent_type(type)
    return :UNKNOWN
end
function equivalent_type(type::ValueType)
    if haskey(value_type_mapping, type.name)
        return value_type_mapping[type.name]
    end

    if !startswith(type.name, "mj")
        @show type.name
    end

    return Symbol(type.name)
end
function equivalent_type(type::CAnonymousType)
    return :ANON
end
function equivalent_type(type::PointerType)
    return Expr(:curly, :Ptr, equivalent_type(type.inner_type))
end
function equivalent_type(type::ArrayType)
    # ToDo use the extents for reshaping arrays
    return Expr(:curly, :NTuple, reduce(*, type.extents), equivalent_type(type.inner_type))
end
function declare(struct_field::StructFieldDecl)
    # Disallow reserved names
    disallowed_names = Set(("global",))
    name = if struct_field.name in disallowed_names
        "_$(struct_field.name)"
    else
        struct_field.name
    end

    return Expr(:(::), Symbol(name), equivalent_type(struct_field.type))
end

function declare(s::StructDecl)
    body = Expr(:block, (declare(f) for f in s.fields)...)
    Expr(:struct, false, Symbol(s.name), body)
end

function declare(s::CAnonymousType)
    error("Should be handled before creating declaration.")
end

function declare_file(filepath, contents; exports = String[], packages = String[])
    remove_line_comments = r"#=.*=#\s*"
    open(filepath, "w") do io
        println(io, """
        # This file is automatically generated and should not be manually edited.
        """)

        for p in packages
            println(io, "using $p")
        end
        println(io)
        
        for c in contents
            expr_unfiltered = string(declare(c))
            expr = replace(expr_unfiltered, remove_line_comments=>"")

            println(io, expr)
            println(io)
        end

        if length(exports) > 0
            print(io, "export ")
            join(io, exports, ", ")
        end
    end
end