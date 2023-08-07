cd(@__DIR__)
using Pkg
Pkg.activate("..")

using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
using MuJoCo

include("../../src/lyceum-vis/glfw.jl")

# Load model and data
m = load_xml("../../models/cartpole.xml")
d = init_data(m)

# Mouse interaction
button_left = false
button_middle = false
button_right = false
lastx = 0.0
lasty = 0.0


