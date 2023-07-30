cd(@__DIR__)
using Pkg
Pkg.activate("../")

using MuJoCo
using MuJoCo.LibMuJoCo


# Load model and data for testing
m = mj_loadXML("cartpole.xml", C_NULL, C_NULL, C_NULL)
d = mj_makeData(m)
