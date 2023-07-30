cd(@__DIR__)
using Pkg
Pkg.activate("../")

using MuJoCo
using MuJoCo.LibMuJoCo

xmlpath = "cartpole.xml"
mjVFS = nothing
err = "Could not load binary model"
err_sz = sizeof(err)

mj_loadXML(xmlpath, mjVFS, err, err_sz)