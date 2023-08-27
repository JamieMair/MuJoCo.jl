# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

function pausestep!(p::PhysicsState)
    m, d = p.model, p.data
    LibMuJoCo.mjv_applyPerturbPose(m, d, p.pert, 1)
    forward!(m, d)
    return p
end

function forwardstep!(p::PhysicsState)
    m, d = p.model, p.data
    fill!(d.xfrc_applied, 0)
    LibMuJoCo.mjv_applyPerturbPose(m, d, p.pert, 0)
    LibMuJoCo.mjv_applyPerturbForce(m, d, p.pert)
    step!(m, d)
    return p
end


############ EngineMode ############

# required
forwardstep!(p, ::EngineMode) = error("must implement")

supportsreverse(::EngineMode) = false
function reversestep!(p, m::EngineMode)
    supportsreverse(m) && error("supportsreverse was true but reversestep! undefined")
    return p
end

# optional
nameof(m::EngineMode) = string(Base.nameof(typeof(m)))
setup!(ui, p, ::EngineMode) = ui
teardown!(ui, p, ::EngineMode) = ui
# reset!(p, ::EngineMode) = (reset!(p.model); p)   # TODO: reset! is highly model-dependent
pausestep!(p, ::EngineMode) = pausestep!(p)
prepare!(ui, p, ::EngineMode) = ui
modeinfo(io1, io2, ui, p, ::EngineMode) = nothing
handlers(ui, p, ::EngineMode) = EventHandler[]


############ PassiveDynamics ############

struct PassiveDynamics <: EngineMode end
forwardstep!(p::PhysicsState, ::PassiveDynamics) = forwardstep!(p)


############ Controller ############

mutable struct Controller{F} <: EngineMode
    controller::F
    realtimefactor::Float64
end
Controller(controller) = Controller(controller, 1.0)

function teardown!(ui::UIState, p::PhysicsState, x::Controller)
    d = p.data
    fill!(d.ctrl, 0)
    fill!(d.qfrc_applied, 0)
    fill!(d.xfrc_applied, 0)
    return ui
end

function forwardstep!(p::PhysicsState, x::Controller)
    m, d = p.model, p.data
    dt = @elapsed x.controller(m, d)
    x.realtimefactor = timestep(p.model) / dt
    return forwardstep!(p)
end

function modeinfo(io1, io2, ui::UIState, p::PhysicsState, x::Controller)
    println(io1, "Realtime Factor")
    @printf io2 "%.2fx\n" x.realtimefactor
    return nothing
end