# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

# Anything commented out is a function we have copied but not yet changed. Some of these will not be required in our final version and can be deleted.

# abstract type ViewerMode end # Previously EngineMode

mutable struct PhysicsState
    model::Model
    data::Data
    pert::VisualiserPerturb
    elapsedsim::Float64
    timer::RateTimer
    lock::ReentrantLock # TODO: Check what we need this for

    function PhysicsState(model::Model, data::Data)
        pert = VisualiserPerturb()
        new(model, data, pert, 0, RateTimer(), ReentrantLock())
    end
end

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

mutable struct MuJoCoViewer
    phys::PhysicsState
    manager::WindowManager
    ui::UIState
    should_close::Bool

    # For button/mouse callbacks
    ffmpeghandle::Maybe{Base.Process}
    framebuf::Vector{UInt8}
    videodst::Maybe{String}
    min_refreshrate::Int

    # Event handlers
    handlers::Vector{EventHandler}
end

"""
    MuJoCoViewer(m::Model, d::Data; show_window=true)

Initialise a visualiser for a given MuJoCo model

This effectively copies initialisation of `Engine(...)` from `LyceumMuJoCoViz.jl`.
"""
function MuJoCoViewer(m::Model, d::Data; show_window=true)

    # Store the physics state
    phys = PhysicsState(m, d)

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

    # For button/mouse callbacks
    ffmpeghandle = nothing
    framebuf = UInt8[]
    videodst = nothing
    min_refreshrate = min(map(GetRefreshRate, GLFW.GetMonitors())..., MIN_REFRESHRATE)

    v = MuJoCoViewer(
        phys, manager, ui, false,
        ffmpeghandle,
        framebuf,
        videodst,
        min_refreshrate,
        EventHandler[]
    )

    # Register event handlers
    v.handlers = handlers(v)
    register!(manager, v.handlers...)

    return v
end

# mutable struct Engine{T,M}
#     phys::PhysicsState{T}
#     ui::UIState
#     mngr::WindowManager
#     handlers::Vector{EventHandler}

#     modes::M
#     modehandlers::Vector{EventHandler}
#     curmodeidx::Int

#     ffmpeghandle::Maybe{Base.Process}
#     framebuf::Vector{UInt8}
#     videodst::Maybe{String}
#     min_refreshrate::Int

# TODO: We'll replace the Engine type with MuJoCoViewer.
#     function Engine(windowsize::NTuple{2,Integer}, model::Union{MJSim,AbstractMuJoCoEnvironment}, modes::Tuple{Vararg{EngineMode}})
#         window = create_window(windowsize..., "LyceumMuJoCoViz")
#         try
#             phys = PhysicsState(model)
#             ui = UIState()
#             mngr = WindowManager(window)

#             mjv_defaultScene(ui.scn)
#             mjv_defaultCamera(ui.cam)
#             mjv_defaultOption(ui.vopt)
#             mjr_defaultContext(ui.con)
#             mjv_defaultFigure(ui.figsensor)

#             sim = getsim(model)
#             mjv_makeScene(sim.m, ui.scn, MAXGEOM) # TODO calculate MAXGEOM form model and moes
#             mjr_makeContext(sim.m, ui.con, FONTSCALE)

#             alignscale!(ui, sim)
#             init_figsensor!(ui.figsensor)

#             e = new{typeof(model),typeof(modes)}(
#                 phys,
#                 ui,
#                 mngr,
#                 EventHandler[],

#                 modes,
#                 handlers(ui, phys, first(modes)),
#                 1,

#                 nothing,
#                 UInt8[],
#                 nothing,
#                 min(map(GetRefreshRate, GLFW.GetMonitors())..., MIN_REFRESHRATE),
#             )

#             e.handlers = handlers(e)
#             register!(mngr, e.handlers...)

#             return e
#         catch e
#             GLFW.DestroyWindow(window)
#             rethrow(e)
#         end
#     end
# end
