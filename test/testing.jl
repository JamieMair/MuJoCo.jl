cd(@__DIR__)
using Pkg
Pkg.activate("../")

using MuJoCo

xml = "../models/cartpole.xml"
m = load_xml(xml)

# This works
println("Number of states: ", m.nq + m.nv)

# TODO: Currently fails unless MuJoCo.LibMuJoCo is explicitly loaded
m.opt # Struct should contain useful info like time step, gravity, etc. Generator looking for main.LibMuJoCo.mjOption_
m.vis # same error as m.opt, generator looking for Main.LibMuJoCo.mjVisual_
m.stat # same error as m.opt, generator looking for Main.LibMuJoCo.mjStatistic_

# TODO: Currently fails regardless
d = init_data(m)