using MuJoCo_jll
using Git
cd(@__DIR__)

# Clone the mujoco directory here
if !isdir("./mujoco")
    run(`$(git()) clone https://github.com/deepmind/mujoco.git`)
end

mj_version = "2.3.7"
cd("./mujoco")
run(`$(git()) checkout tags/$(mj_version)`)
cd("../")

include_dir = normpath(MuJoCo_jll.artifact_dir, "include")
mujoco_headers_dir = joinpath(include_dir, "mujoco")

mujoco_dir = normpath(joinpath("$(@__DIR__)/mujoco"))
introspect_init_file_path = joinpath(mujoco_dir, "introspect", "__init__.py")
if !isfile(introspect_init_file_path)
    open(introspect_init_file_path, 'w') do io
        println(io, "from .functions import FUNCTIONS")
        println(io, "from .structs import STRUCTS")
        println(io, "from .enums import ENUMS")
    end
end


using PyCall
using JuliaFormatter
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
    ast_module.StructFieldDecl => StructFieldDecl,
)

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
        if py"isinstance"(obj, k)
            prop_values = map(fieldnames(v)) do p
                return recursive_convert(getproperty(obj, p))
            end
            return v(prop_values...)
        end
    end

    error("Could not find mapping for $obj")
end

enum_mapping = recursive_convert(pyimport("introspect.enums").ENUMS)
function_mapping = recursive_convert(pyimport("introspect.functions").FUNCTIONS)
struct_mapping = recursive_convert(pyimport("introspect.structs").STRUCTS)

cd(@__DIR__)
module_path = normpath(abspath("./Core"))
wrapper_folder = joinpath(module_path, "wrappers")
if !isdir(module_path)
    mkdir(module_path)
end
if !isdir(wrapper_folder)
    mkdir(wrapper_folder)
end

module_file = joinpath(module_path, "core.jl")
open(module_file, "w") do io
    println(io, "module Core")
    println(io)
    println(io, "# Wrappers")
    println(io, raw"""include("wrappers/enums.jl")""")
    println(io)
    println(io, "end")
end

enum_file = joinpath(wrapper_folder, "enums.jl")
declare_file(enum_file, values(enum_mapping); exports = keys(enum_mapping), packages=["CEnum"])
format_file(enum_file)


