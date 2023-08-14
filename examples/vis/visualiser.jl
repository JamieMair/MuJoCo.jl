# Many elements of this file are taken from https://github.com/Lyceum/LyceumMuJoCoViz.jl
# It is a work-in-progress

import Base.RefValue

import MuJoCo.LibMuJoCo
import MuJoCo.LibMuJoCo: Model, Data
import MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
import MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render
using MuJoCo.Visualiser
using StaticArrays

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
Base.@kwdef mutable struct UIState
    scn::VisualiserScene = VisualiserScene()
    cam::VisualiserCamera = VisualiserCamera()
    vopt::VisualiserOption = VisualiserOption()
    con::RendererContext = RendererContext()
    figsensor::VisualiserFigure = VisualiserFigure()

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
    return ui
end

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

# # From functions.jl in LyceumMuJoCoViz
# # TODO: Can't change this because ui.cam is an immutable struct
function alignscale!(ui::UIState, m::Model)
    ui.cam.lookat .= m.stat.center
    ui.cam.distance = 1.5 * m.stat.extent
    ui.cam.type = LibMuJoCo.mjCAMERA_FREE
    return ui
end

# # From util.jl in LyceumMuJoCoViz
function str2vec(s::String, len::Int)
    str = zeros(UInt8, len)
    str[1:length(s)] = codeunits(s)
    return str
end
function set_string!(buffer::AbstractArray, s::String)
    buffer_size = length(buffer)
    s_length = length(s)
    @assert s_length < buffer_size "The buffer for this string is only $buffer_size large, but tried to put in a string of length $s_length"
    buffer[1:s_length] = codeunits(s)
    buffer[s_length+1] = zero(eltype(buffer)) # Assume zero-terminated string
    nothing
end

function init_figsensor!(figsensor::VisualiserFigure)
    figsensor.flg_extend = 1
    figsensor.flg_barplot = 1
    set_string!(figsensor.title, "Sensor Data")
    set_string!(figsensor.title, "%.0f")
    figsensor.gridsize .= @SVector [2, 3]
    figsensor.range .= @SMatrix [0;1;;-1;1]
    return figsensor
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
    ui.refreshrate = GetRefreshRate()
    ui.lastrender = time()

    # Create scene and context
    LibMuJoCo.mjv_makeScene(m.internal_pointer, ui.scn.internal_pointer, MAXGEOM)
    LibMuJoCo.mjr_makeContext(m.internal_pointer, ui.con.internal_pointer, LibMuJoCo.mjFONTSCALE_150)

    # The remaining comments are notes on what to add when incorporating LyceumMuJoCoViz

    # TODO: Add these in once ui.cam[] and others are actually mutable
    alignscale!(ui, m)
    init_figsensor!(ui.figsensor)

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