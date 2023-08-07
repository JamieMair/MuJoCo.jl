# From Jamie's basic render
function alloc(::Type{T}) where {T}
    return Ptr{T}(Libc.malloc(sizeof(T)))
end

# Form Lyceum types.jl
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
function render(mngr::WindowManager, ui::UIState)
    w, h = GLFW.GetFramebufferSize(mngr.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    mjr_render(rect, ui.scn, ui.con)
    # ui.showinfo && overlay_info(rect, e) # TODO: Add in the info later
    GLFW.SwapBuffers(mngr.state.window)
    return
end