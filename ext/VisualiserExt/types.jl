# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

abstract type EngineMode end

mutable struct PhysicsState
    model::Model
    data::Data
    pert::VisualiserPerturb
    elapsedsim::Float64
    timer::RateTimer
    lock::ReentrantLock

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

mutable struct Engine{M}
    phys::PhysicsState
    manager::WindowManager
    ui::UIState

    # Engine modes
    modes::M
    modehandlers::Vector{EventHandler}
    curmodeidx::Int

    # For button/mouse callbacks
    ffmpeghandle::Maybe{Base.Process}
    framebuf::Vector{UInt8}
    videodst::Maybe{String}
    min_refreshrate::Int

    # Event handlers
    handlers::Vector{EventHandler}
end

function Engine(
    windowsize::NTuple{2,Integer}, 
    m::Model, 
    d::Data,
    modes::Tuple{Vararg{EngineMode}},
)    

    # Store physics sate and visualisation interface
    phys = PhysicsState(m, d)
    ui = UIState()

    # Create the window and manager
    window = create_window(windowsize..., "MuJoCo.jl")
    manager = WindowManager(window)

    # Create scene and context
    LibMuJoCo.mjv_makeScene(m, ui.scn, MAXGEOM)
    LibMuJoCo.mjr_makeContext(m, ui.con, LibMuJoCo.mjFONTSCALE_150)

    alignscale!(ui, m)
    init_figsensor!(ui.figsensor)

    # For button/mouse callbacks
    ffmpeghandle = nothing
    framebuf = UInt8[]
    videodst = nothing
    min_refreshrate = min(map(GetRefreshRate, GLFW.GetMonitors())..., MIN_REFRESHRATE)
    
    # Build the engine
    e = Engine{typeof(modes)}(
        phys, manager, ui,
        modes,
        handlers(ui, phys, first(modes)),
        1,
        ffmpeghandle,
        framebuf,
        videodst,
        min_refreshrate,
        EventHandler[]
    )

    e.handlers = handlers(e)
    register!(manager, e.handlers...)

    return e
end
