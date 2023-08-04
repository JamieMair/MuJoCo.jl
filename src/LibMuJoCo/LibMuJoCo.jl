module LibMuJoCo
using MuJoCo_jll
export MuJoCo_jll
using CEnum
include("consts.jl")
include("enums.jl")
include("structs.jl")
include("functions.jl")
include("wrappers.jl") # TODO: Had to add this in manually
for name in names(@__MODULE__(); all = true), prefix in PREFIXES
    if startswith(string(name), prefix)
        @eval export $name
    end
end
end
