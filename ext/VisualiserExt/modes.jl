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
reset!(p, ::EngineMode) = (MuJoCo.reset!(p.model, p.data); p)
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

############ Trajectory ############

mutable struct Trajectory{TR<:AbstractVector{<:AbstractMatrix}} <: EngineMode
    trajectories::TR
    k::Int
    t::Int

    burstmode::Bool
    bf_idx::Int
    bf_range::LinRange{Float64}
    bg_idx::Int
    bg_range::LinRange{Float64}
    doppler::Bool
end
function Trajectory{TR}(trajectories) where {TR<:AbstractVector{<:AbstractMatrix}}
    Trajectory{TR}(
        trajectories, 1, 1, 
        false, 1, LinRange(0, 1, 2), 1, LinRange(0, 1, 2), false
    )
end
Trajectory(trajectories::AbstractVector{<:AbstractMatrix}) = Trajectory{typeof(trajectories)}(trajectories)
Trajectory(trajectories::AbstractMatrix) = Trajectory([trajectories])

getT(m::Trajectory) = size(m.trajectories[m.k], 2)
gettraj(m::Trajectory) = m.trajectories[m.k]

function setup!(ui::UIState, p::PhysicsState, m::Trajectory)
    setburstmodeparams!(m, p)
    setstate!(m, p)
    return ui
end

function reset!(p::PhysicsState, m::Trajectory)
    m.t = 1 
    setstate!(m, p)
    return p
end

function forwardstep!(p::PhysicsState, m::Trajectory)
    m.t = inc(m.t, 1, getT(m))
    setstate!(m, p)
    return p
end

supportsreverse(::Trajectory) = true
function reversestep!(p::PhysicsState, m::Trajectory)
    m.t = dec(m.t, 1, getT(m))
    setstate!(m, p)
    return p
end

function prepare!(ui::UIState, p::PhysicsState, m::Trajectory)
    if m.burstmode
        bf = round(Int, m.bf_range[m.bf_idx])
        bg = m.bg_range[m.bg_idx]
        burst!(ui, p, gettraj(m), bf, m.t, gamma = bg, doppler = m.doppler)
    end
    return ui
end

function modeinfo(io1, io2, ui::UIState, p::PhysicsState, m::Trajectory)
    println(io1, "t")
    println(io2, "$(m.t)/$(getT(m))")
    println(io1, "Trajectory ID")
    println(io2, "$(m.k)/$(length(m.trajectories))")

    n = length(m.bf_range)
    println(io1, "Burst Mode")
    println(io2, m.burstmode ? "Enabled" : "Disabled")
    println(io1, "Burst Factor")
    println(io2, "$(m.bf_idx)/$n")
    println(io1, "Burst Gamma")
    println(io2, "$(m.bg_idx)/$n")
    println(io1, "Burst Doppler")
    println(io2, m.doppler ? "Enabled" : "Disabled")

    return nothing
end

function handlers(ui::UIState, p::PhysicsState, m::Trajectory)
    return let ui=ui, p=p, m=m
        [
            onkey(GLFW.KEY_R, MOD_CONTROL, what = "Toggle reverse") do s, ev
                if ispress_or_repeat(ev.action)
                    ui.reversed = !ui.reversed
                    p.timer.rate *= -1
                    resettime!(p)
                end
            end,

            onevent(
                KeyEvent,
                when = describe(GLFW.KEY_LEFT),
                what = "Step backwards when paused (hold SHIFT for $(SHIFTSTEPSPERKEY) steps)"
            ) do s, ev
                if ui.paused && ev.key === GLFW.KEY_LEFT && ispress_or_repeat(ev.action)
                    steps = s.shift ? 50 : 1
                    for _ = 1:steps
                        reversestep!(p, m)
                    end
                end
            end,

            onkey(GLFW.KEY_UP, what = "Cycle forwards through trajectories") do s, ev
                if ispress_or_repeat(ev.action)
                    m.k = inc(m.k, 1, length(m.trajectories))
                    ax = axes(gettraj(m), 2)
                    checkbounds(Bool, ax, m.t) || (m.t = last(ax))
                    setburstmodeparams!(m, p)
                    setstate!(m, p)
                end
            end,

            onkey(GLFW.KEY_DOWN, what = "Cycle backwards through trajectories") do s, ev
                if ispress_or_repeat(ev.action)
                    m.k = dec(m.k, 1, length(m.trajectories))
                    ax = axes(gettraj(m), 2)
                    checkbounds(Bool, ax, m.t) || (m.t = last(ax))
                    setburstmodeparams!(m, p)
                    setstate!(m, p)
                end
            end,

            onkey(GLFW.KEY_B, MOD_CONTROL, what = "Toggle burst mode") do s, ev
                ispress_or_repeat(ev.action) && (m.burstmode = !m.burstmode)
            end,

            onkey(GLFW.KEY_D, MOD_CONTROL, what = "Toggle burst mode doppler effect") do s, ev
                ispress_or_repeat(ev.action) && (m.doppler = !m.doppler)
            end,

            onscroll(MOD_CONTROL, what = "Change burst factor") do s, ev
                m.bf_idx = clamp(m.bf_idx + ev.dy, 1, length(m.bf_range))
            end,

            onscroll(MOD_SHIFT, what = "Change burst decay rate") do s, ev
                m.bg_idx = clamp(m.bg_idx + ev.dy, 1, length(m.bg_range))
            end,
        ]
    end
end

function setburstmodeparams!(m::Trajectory, p::PhysicsState)
    steps = 40 # n scroll increments

    # Params calibrated on cartpole and scaled accordingly
    # Rendering up to bf0_max states for a len0 length trajectory
    # with a timestep of dt0 and gamma0 looks reasonalbe.
    len0 = 100
    bf0_max = 50
    gamma0 = 0.85
    dt0 = 0.01

    dt = timestep(p.model)
    len = size(gettraj(m), 2)

    bf_max = round(Int, bf0_max * (len / len0) * (dt / dt0))
    gamma = gamma0^(dt / dt0)

    m.bf_range = LinRange{Float64}(1, bf_max, steps) # render between 1 and bf_max states
    m.bg_range = LinRange{Float64}(gamma, 1, steps)

    m.bf_idx = round(Int, steps / 2)
    m.bg_idx = round(Int, steps / 2)

    return m
end

@inline function setstate!(m::Trajectory, p::PhysicsState)
    set_physics_state!(p.model, p.data, view(gettraj(m), :, m.t))
    forward!(p.model, p.data)
    return m
end

############ Util ############


function burst!(
    ui::UIState,
    p::PhysicsState,
    states::AbstractMatrix,
    n::Integer,
    t::Integer;
    gamma::Real = 0.9995,
    alphamin::Real = 0.05,
    alphamax::Real = 0.55,
    doppler::Bool = true,
)

    # Check inputs
    T = size(states, 2)
    (T >= n > 0) || error("n must be in range [1, size(states, 2)]")
    (0 < t) || error("t must be >= 0")
    (0 < gamma <= 1) || error("gamma must be in range (0, 1)")
    (0 < alphamin <= 1) || error("alphamin must be in range (0, 1]")
    (0 < alphamax <= 1) || error("alphamin must be in range (0, 1]")

    scn = ui.scn
    n = min(n, fld(MAXGEOM, p.model.ngeom))
    geoms = unsafe_wrap(Array, scn.geoms, scn.maxgeom)

    function color!(tprime, from)
        for i = from:scn.ngeom
            geom = @inbounds geoms[i]
            if geom.category == Int(LibMuJoCo.mjCAT_DYNAMIC)
                dist = abs(tprime - t)
                r, g, b, alpha0 = geom.rgba

                alpha = alpha0 * gamma^dist

                if doppler
                    g = 0
                    if tprime < t
                        beta = (dist + 1) / t / 2 + 0.5
                        r = beta * r
                        b = (1 - beta) * b
                    else
                        beta = dist / (T - t) / 2 + 0.5
                        r = (1 - beta) * r
                        b = beta * b
                    end
                    r = clamp(r, 0, 1)
                    b = clamp(b, 0, 1)
                end

                @set!! geom.rgba = NTuple{4,Cfloat}((r, g, b, alpha))
                geoms[i] = geom
            end
        end
    end

    set_physics_state!(p.model, p.data, view(states, :, t))
    forward!(p.model, p.data)
    mjv_updateScene(
        p.model,
        p.data,
        ui.vopt,
        p.pert,
        ui.cam,
        LibMuJoCo.mjCAT_ALL,
        scn,
    )

    if n > 1
        from = scn.ngeom + 1
        for tprime in Iterators.map(x -> round(Int, x), LinRange(1, T, n))
            if tprime != t
                set_physics_state!(p.model, p.data, view(states, :, tprime))
                forward!(p.model, p.data)
                mjv_addGeoms(
                    p.model,
                    p.data,
                    ui.vopt,
                    p.pert,
                    LibMuJoCo.mjCAT_DYNAMIC,
                    scn,
                )
                color!(tprime, from)
            end
            from = scn.ngeom + 1
        end
    end

    # Reset the model to the state it had when it was passed in
    set_physics_state!(p.model, p.data, view(states, :, t))
    forward!(p.model, p.data)

    return ui
end
