# This script tries to add in docstrings for the underlying library
using Git
using MuJoCo_jll
cd(@__DIR__)
# Clone the mujoco directory here
if !isdir("./mujoco")
    run(`$(git()) clone https://github.com/deepmind/mujoco.git`)
end

mj_version = "3.1.6"
cd("./mujoco")
run(`$(git()) checkout tags/$(mj_version)`)
cd("../")

include_dir = normpath(MuJoCo_jll.artifact_dir, "include")
mujoco_headers_dir = joinpath(include_dir, "mujoco")

mujoco_dir = normpath(joinpath("$(@__DIR__)/mujoco"))
introspect_init_file_path = joinpath(mujoco_dir, "introspect", "__init__.py")
if !isfile(introspect_init_file_path)
    open(introspect_init_file_path, "w") do io
        println(io, "from .functions import FUNCTIONS")
        println(io, "from .structs import STRUCTS")
        println(io, "from .enums import ENUMS")
    end
end



using PyCall
include("ast_nodes.jl")

path_vector = pyimport("sys")."path"
module_path = normpath(mujoco_dir)
if path_vector[1] != module_path 
    pushfirst!(PyVector(pyimport("sys")."path"), module_path)
end
ast_module = pyimport("introspect.ast_nodes")
const type_mappings = Dict(
    ast_module.ValueType => ValueType,
    ast_module.ArrayType => ArrayType,
    ast_module.PointerType => PointerType,
    ast_module.FunctionParameterDecl => FunctionParameterDecl,
    ast_module.FunctionDecl => FunctionDecl,
    ast_module._EnumDeclValues => _EnumDeclValues,
    ast_module.EnumDecl => EnumDecl,
    ast_module.StructFieldDecl => StructFieldDecl,
    ast_module.AnonymousStructDecl => AnonymousStructDecl,
    ast_module.AnonymousUnionDecl => AnonymousUnionDecl,
    ast_module.StructDecl => StructDecl,
)
py"""
def checktype(obj, type_class):
    return type(obj) is type_class
"""
const pychecktype = py"checktype"

function recursive_convert(obj)
    return obj # By default return itself
end
function recursive_convert(obj::Dict)
    new_keys = recursive_convert.(keys(obj))
    new_values = recursive_convert.(values(obj))
    
    return Dict(
        (k=>v for (k, v) in zip(new_keys, new_values))...
    )
end
function recursive_convert(obj::PyCall.PyObject)
    for (k, v) in type_mappings
        if pychecktype(obj, k)
            prop_values = map(fieldnames(v)) do p
                return recursive_convert(getproperty(obj, p))
            end
            return v(prop_values...)
        end
    end

    error("Could not find mapping for $obj")
end
function recursive_convert(obj::Tuple)
    return Tuple(recursive_convert(t) for t in obj)
end

const enum_mapping = recursive_convert(pyimport("introspect.enums").ENUMS);
const struct_mapping = recursive_convert(pyimport("introspect.structs").STRUCTS);
const function_mapping = recursive_convert(pyimport("introspect.functions").FUNCTIONS);

function construct_struct_docs(expr)
    @assert expr.head == :struct
    struct_name = string(expr.args[2])
    if endswith(struct_name, "_")
        struct_name = struct_name[begin:end-1]
    end

    if haskey(struct_mapping, struct_name)
        s = struct_mapping[struct_name]
        io = IOBuffer()
        ndocs = 0
        println(io, "\t$struct_name")
        println(io, "")
        println(io, "# Fields")
        println(io, "")
        for f in s.fields
            if f isa StructFieldDecl
                if !isempty(f.doc)
                    ndocs += 1
                    println(io, "  - **`$(f.name)`**: $(f.doc)")
                end 
            end
        end
        docs = ndocs == 0 ? "\t$struct_name" : String(take!(io))
        return Expr(:macrocall, :(Core.var"@doc"), LineNumberNode(1), docs, expr)
    else
        return expr
    end
end