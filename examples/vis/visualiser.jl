import MuJoCo.LibMuJoCo
import MuJoCo.LibMuJoCo: Model, Data
import MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
import MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render

using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
using Observables: AbstractObservable, Observable, on, off

const Maybe{T} = Union{T, Nothing} # From LyceumBase.jl
const MAXGEOM = 10000 # preallocated geom array in mjvScene

include("glfw.jl")

# ----------------------------------------------------------------------------------

function alloc(::Type{T}) where {T}
    return Ptr{T}(Libc.malloc(sizeof(T)))
end

# From Lyceum types.jl
# TODO: Move back into a types file
Base.@kwdef mutable struct UIState
    scn::Ptr{mjvScene} = alloc(mjvScene)
    cam::Ptr{mjvCamera} = alloc(mjvCamera)
    vopt::Ptr{mjvOption} = alloc(mjvOption)
    con::Ptr{mjrContext} = alloc(mjrContext)
    figsensor::Ptr{mjvFigure} = alloc(mjvFigure)

    showinfo::Bool = true
    showsensor::Bool = false
    speedmode::Bool = false
    speedfactor::Float64 = 1 / 10

    reversed::Bool = false
    paused::Bool = true
    shouldexit::Bool = false

    reward::Float64 = 0

    lastrender::Float64 = 0
    refreshrate::Float64 = 0
    realtimerate::Float64 = 0
    io1::IOBuffer = IOBuffer()
    io2::IOBuffer = IOBuffer()

    lock::ReentrantLock = ReentrantLock()
end

"""
    init_ui!(ui::UIState)

Initialise an instance of `UIState` with defaults
"""
function init_ui!(ui::UIState)
    LibMuJoCo.mjv_defaultScene(ui.scn)
    LibMuJoCo.mjv_defaultCamera(ui.cam)
    LibMuJoCo.mjv_defaultOption(ui.vopt)
    LibMuJoCo.mjr_defaultContext(ui.con)
    LibMuJoCo.mjv_defaultFigure(ui.figsensor)
    return ui
end

# Adapted from LyceumMuJoCoViz.jl
function render(manager::WindowManager, ui::UIState)
    w, h = GLFW.GetFramebufferSize(manager.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    mjr_render(rect, ui.scn, ui.con)
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
    ui = init_ui!(UIState())

    # TODO: Add these
    # ui.refreshrate = GetRefreshRate()
    # ui.lastrender = time()

    # Create scene and context
    LibMuJoCo.mjv_makeScene(m.internal_pointer, ui.scn, MAXGEOM)
    LibMuJoCo.mjr_makeContext(m.internal_pointer, ui.con, LibMuJoCo.mjFONTSCALE_150)

    # TODO: Add equivalent of these once we have nice array handling
    # alignscale!(ui, sim)
    # init_figsensor!(ui.figsensor)

    # TODO: add handlers to the struct, see defaulthandlers.jl
    # handlers = handlers(e)
    # register!(mngr, handlers...)

    # TODO: Add frame buffer and framerate stuff from engine
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
        viewer.ui.vopt, 
        C_NULL, 
        viewer.ui.cam, 
        LibMuJoCo.mjCAT_ALL, 
        viewer.ui.scn
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

    LibMuJoCo.mjv_freeScene(viewer.ui.scn)
    LibMuJoCo.mjr_freeContext(viewer.ui.con)

    Libc.free(viewer.ui.cam)
    Libc.free(viewer.ui.vopt)
    Libc.free(viewer.ui.scn)
    Libc.free(viewer.ui.con)

    GLFW.DestroyWindow(viewer.manager.state.window)

    return nothing
end