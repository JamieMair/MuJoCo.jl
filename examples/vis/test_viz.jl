cd(@__DIR__)
using Pkg
Pkg.activate("..")

using MuJoCo

########################################################################
# Extras for Lyceum GLFW support

import MuJoCo.LibMuJoCo
import MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
import MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render

using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
using Observables: AbstractObservable, Observable, on, off

const Maybe{T} = Union{T, Nothing}

include("glfw.jl")
include("utils.jl")

const MAXGEOM = 10000 # preallocated geom array in mjvScene


########################################################################
# Test visualiser: follow recipe from Lyecum Engine type

# Load model and data
m = load_xml("../../models/humanoid.xml")
d = init_data(m)

# Create a window and manager
window = create_window(default_windowsize()..., "MuJoCo.jl")
mngr = WindowManager(window)
GLFW.ShowWindow(mngr.state.window)

# Initialise visualisation data structures
ui = init_ui!(UIState())

# Create scene and context
# TODO: Should write wrappers so this is nice
LibMuJoCo.mjv_makeScene(m.internal_pointer, ui.scn, MAXGEOM)
LibMuJoCo.mjr_makeContext(m.internal_pointer, ui.con, LibMuJoCo.mjFONTSCALE_150)

# TODO: Add equivalent of these once we have nice array handling
# alignscale!(ui, sim)
# init_figsensor!(ui.figsensor)

# Loop and simulate for now
while !GLFW.WindowShouldClose(mngr.state.window)

    # TODO: nsteps = Int(ceil((1.0/60.0) / m.opt.timestep))
    nsteps = Int(ceil((1.0/60.0) / 0.01))
    for _ in 1:nsteps
        step!(m, d)
    end

    # Update scene and render
    LibMuJoCo.mjv_updateScene(m.internal_pointer, d.internal_pointer, ui.vopt, C_NULL, ui.cam, LibMuJoCo.mjCAT_ALL, ui.scn)
    render(mngr, ui)
    GLFW.PollEvents()
end

# Free all the variables
function close_viewer!(ui,mngr)

    LibMuJoCo.mjv_freeScene(ui.scn)
    LibMuJoCo.mjr_freeContext(ui.con)

    Libc.free(ui.cam)
    Libc.free(ui.vopt)
    Libc.free(ui.scn)
    Libc.free(ui.con)

    GLFW.DestroyWindow(mngr.state.window)
end

LibMuJoCo.mj_deleteModel(m.internal_pointer)
LibMuJoCo.mj_deleteData(d.internal_pointer)
close_viewer!(ui,mngr)
