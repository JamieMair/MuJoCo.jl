# Many elements of this file are taken from https://github.com/Lyceum/LyceumMuJoCoViz.jl
# It is a work-in-progress

import Base.RefValue

if isdefined(Base, :get_extension)
    import MuJoCo.LibMuJoCo: Model, Data
    import MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
    import MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render
    using MuJoCo.Visualiser
    using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
    using Observables: AbstractObservable, Observable, on, off
else
    import ..MuJoCo.LibMuJoCo: Model, Data
    import ..MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
    import ..MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render
    using ..MuJoCo.Visualiser
    using ..GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
    using ..Observables: AbstractObservable, Observable, on, off
end

const Maybe{T} = Union{T, Nothing} # From LyceumBase.jl
const MAXGEOM = 10000 # preallocated geom array in mjvScene
const MIN_REFRESHRATE = 30 # minimum rate when sim cannot run at the native refresh rate

const RES_HD = (1280, 720)
const RES_FHD = (1920, 1080)
const RES_XGA = (1024, 768)
const RES_SXGA = (1280, 1024)

include("util.jl")
include("glfw.jl")
include("ratetimer.jl")
include("types.jl")
include("functions.jl")
include("defaulthandlers.jl")


# ----------------------------------------------------------------------------------


# Adapted from LyceumMuJoCoViz.jl
function render(manager::WindowManager, ui::UIState)
    w, h = GLFW.GetFramebufferSize(manager.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    mjr_render(rect, ui.scn.internal_pointer, ui.con.internal_pointer)
    # ui.showinfo && overlay_info(rect, e) # TODO: Add in the info later
    GLFW.SwapBuffers(manager.state.window)
    return
end

"""
    render!(v::MuJoCoViewer, m::Model, d::Data)

Render the viewer given current model and data.

# TODO: Add all extra stuff equivalent to `runui(e::Engine)` in `LyceumMuJoCoViz.jl` file
"""
function render!(v::MuJoCoViewer, m::Model, d::Data)
    LibMuJoCo.mjv_updateScene(
        m.internal_pointer, 
        d.internal_pointer, 
        v.ui.vopt.internal_pointer, 
        C_NULL, 
        v.ui.cam.internal_pointer, 
        LibMuJoCo.mjCAT_ALL, 
        v.ui.scn.internal_pointer
    )
    render(v.manager, v.ui)
    GLFW.PollEvents()
    v.should_close = v.ui.shouldexit | GLFW.WindowShouldClose(v.manager.state.window)

    return nothing
end

"""
    close_viewer!(v::MuJoCoViewer)

Close the viewer when done.
"""
function close_viewer!(v::MuJoCoViewer)
    GLFW.DestroyWindow(v.manager.state.window)
    return nothing
end
