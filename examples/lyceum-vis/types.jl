abstract type EngineMode end

mutable struct PhysicsState{T<:Union{MJSim,AbstractMuJoCoEnvironment}}
    model::T
    pert::RefValue{mjvPerturb}
    elapsedsim::Float64
    timer::RateTimer
    lock::ReentrantLock

    function PhysicsState(model::Union{MJSim,AbstractMuJoCoEnvironment})
        pert = Ref(mjvPerturb())
        mjv_defaultPerturb(pert)
        new{typeof(model)}(model, pert, 0, RateTimer(), ReentrantLock())
    end
end

Base.@kwdef mutable struct UIState
    scn::RefValue{mjvScene} = Ref(mjvScene())
    cam::RefValue{mjvCamera} = Ref(mjvCamera())
    vopt::RefValue{mjvOption} = Ref(mjvOption())
    con::RefValue{mjrContext} = Ref(mjrContext())
    figsensor::RefValue{mjvFigure} = Ref(mjvFigure())

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

mutable struct Engine{T,M}
    phys::PhysicsState{T}
    ui::UIState
    mngr::WindowManager
    handlers::Vector{EventHandler}

    modes::M
    modehandlers::Vector{EventHandler}
    curmodeidx::Int

    ffmpeghandle::Maybe{Base.Process}
    framebuf::Vector{UInt8}
    videodst::Maybe{String}
    min_refreshrate::Int

    function Engine(windowsize::NTuple{2,Integer}, model::Union{MJSim,AbstractMuJoCoEnvironment}, modes::Tuple{Vararg{EngineMode}})
        window = create_window(windowsize..., "LyceumMuJoCoViz")
        try
            phys = PhysicsState(model)
            ui = UIState()
            mngr = WindowManager(window)

            mjv_defaultScene(ui.scn)
            mjv_defaultCamera(ui.cam)
            mjv_defaultOption(ui.vopt)
            mjr_defaultContext(ui.con)
            mjv_defaultFigure(ui.figsensor)

            sim = getsim(model)
            mjv_makeScene(sim.m, ui.scn, MAXGEOM) # TODO calculate MAXGEOM form model and moes
            mjr_makeContext(sim.m, ui.con, FONTSCALE)

            alignscale!(ui, sim)
            init_figsensor!(ui.figsensor)

            e = new{typeof(model),typeof(modes)}(
                phys,
                ui,
                mngr,
                EventHandler[],

                modes,
                handlers(ui, phys, first(modes)),
                1,

                nothing,
                UInt8[],
                nothing,
                min(map(GetRefreshRate, GLFW.GetMonitors())..., MIN_REFRESHRATE),
            )

            e.handlers = handlers(e)
            register!(mngr, e.handlers...)

            return e
        catch e
            GLFW.DestroyWindow(window)
            rethrow(e)
        end
    end
end
