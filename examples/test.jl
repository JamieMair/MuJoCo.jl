cd(@__DIR__)
using Pkg
Pkg.activate(".")

# TODO: Need to explicitly add our MuJoCo_jll when adding MuJoCo.jl to a new project
using MuJoCo

# Load a model and data
m = load_xml("../models/humanoid.xml")
d = init_data(m)

println(m.opt.timestep)
println(m.opt.gravity)
