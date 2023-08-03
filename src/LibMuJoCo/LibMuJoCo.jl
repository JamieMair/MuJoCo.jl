module LibMuJoCo
using MuJoCo_jll
export MuJoCo_jll
using CEnum
include("consts.jl")
include("enums.jl")
include("structs.jl")
include("functions.jl")
for name = names(@__MODULE__(); all = true), prefix = PREFIXES
    if startswith(string(name), prefix)
        @eval export $name
    end
    end
end