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

include("util.jl")
include("glfw.jl")
include("ratetimer.jl")
include("types.jl")
include("functions.jl")

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

mutable struct MuJoCoViewer
    manager::WindowManager
    ui::UIState
    should_close::Bool
end

"""
    MuJoCoViewer(m::Model, d::Data; show_window=true)

Initialise a visualiser for a given MuJoCo model

#TODO: This should effectively copy initialisation of `Engine(...)` from the `LyceumMuJoCoViz` `types.jl` file.
"""
function MuJoCoViewer(m::Model, d::Data; show_window=true)

    # Create and show window
    window = create_window(default_windowsize()..., "MuJoCo.jl")
    manager = WindowManager(window)
    show_window && GLFW.ShowWindow(manager.state.window)

    # Initialise visualisation data structures
    ui = UIState()
    ui.refreshrate = GetRefreshRate()
    ui.lastrender = time()

    # Create scene and context
    LibMuJoCo.mjv_makeScene(m.internal_pointer, ui.scn.internal_pointer, MAXGEOM)
    LibMuJoCo.mjr_makeContext(m.internal_pointer, ui.con.internal_pointer, LibMuJoCo.mjFONTSCALE_150)

    alignscale!(ui, m)
    init_figsensor!(ui.figsensor)

    # The remaining comments are notes on what to add when incorporating LyceumMuJoCoViz

    # TODO: add handlers to the struct, see defaulthandlers.jl
    # handlers = handlers(e)
    # register!(mngr, handlers...)

    # # TODO: Add frame buffer and framerate stuff from engine as fields for later use
    # nothing,
    # UInt8[],
    # nothing,
    # min(map(GetRefreshRate, GLFW.GetMonitors())..., MIN_REFRESHRATE),

    return MuJoCoViewer(manager, ui, false)
end

"""
    render!(viewer::MuJoCoViewer, m::Model, d::Data)

Render the viewer given current model and data.

# TODO: Add all extra stuff equivalent to `runui(e::Engine)` in `LyceumMuJoCoViz.jl` file
"""
function render!(viewer::MuJoCoViewer, m::Model, d::Data)
    LibMuJoCo.mjv_updateScene(
        m.internal_pointer, 
        d.internal_pointer, 
        viewer.ui.vopt.internal_pointer, 
        C_NULL, 
        viewer.ui.cam.internal_pointer, 
        LibMuJoCo.mjCAT_ALL, 
        viewer.ui.scn.internal_pointer
    )
    render(viewer.manager, viewer.ui)
    GLFW.PollEvents()
    viewer.should_close = GLFW.WindowShouldClose(viewer.manager.state.window)

    return nothing
end

"""
    close_viewer!(ui, manager)

Close the viewer when done.
"""
function close_viewer!(viewer::MuJoCoViewer)
    GLFW.DestroyWindow(viewer.manager.state.window)
    return nothing
end
