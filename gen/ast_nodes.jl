using CEnum

abstract type CDataType end

"""Represents a C type that is neither a pointer type nor an array type."""
struct ValueType <: CDataType
    name::String
    is_const::Bool
    is_volatile::Bool
end

"""Represents a C array type."""
struct ArrayType{T<:CDataType,N,E<:NTuple{N,Int}} <: CDataType
    inner_type::T
    extents::E
end

"""Represents a C pointer type."""
struct PointerType{T<:CDataType} <: CDataType
    inner_type::T
    is_const::Bool
    is_volatile::Bool
    is_restrict::Bool
end

"""Represents a parameter in a function declaration.

Note that according to the C language rule, a function parameter of array
type undergoes array-to-pointer decay, and therefore appears as a pointer
parameter in an actual C AST. We retain the arrayness of a parameter here
since the array's extents are informative.
"""
struct FunctionParameterDecl{T<:CDataType}
    name::String
    type::T
end

"""Represents a function declaration."""
struct FunctionDecl{RT<:CDataType,N,PT<:NTuple{N,FunctionParameterDecl}}
    name::String
    return_type::RT
    parameters::PT
    doc::String
end

struct _EnumDeclValues
    mapping::Dict{String,Int}
end

"""Represents an enum declaration."""
struct EnumDecl
    name::String
    declname::String
    values::Dict{String,Int}
end

abstract type CAnonymousType end

"""Represents a field in a struct or union declaration."""
struct StructFieldDecl{T<:Union{CDataType,CAnonymousType}}
    name::String
    type::T
    doc::String
end

"""Represents an anonymous struct declaration."""
struct AnonymousStructDecl{FT} <: CAnonymousType
    fields::FT
end

"""Represents an anonymous union declaration."""
struct AnonymousUnionDecl{FT} <: CAnonymousType
    fields::FT
end

"""Represents a struct declaration."""
struct StructDecl{FT<:Tuple}
    name::String
    declname::String
    fields::FT
end


## Declarations
function declare(enum::EnumDecl)
    pair_values = [(k, v) for (k, v) in enum.values]
    sort!(pair_values; by=(x->x[2]))
    body = Expr(:block, (Expr(:(=), Symbol(k), v) for (k, v) in pair_values)...)
    enum_name = Expr(:(::), Symbol(enum.name), :Cint)
    Expr(:macrocall, Symbol("@cenum"), LineNumberNode(1), enum_name, body)
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