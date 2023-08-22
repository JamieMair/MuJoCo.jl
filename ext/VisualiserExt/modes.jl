# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

# Anything commented out is a function we have copied but not yet changed. Some of these will not be required in our final version and can be deleted.

function pausestep!(p::PhysicsState)
    m, d = p.model, p.data
    LibMuJoCo.mjv_applyPerturbPose(m.internal_pointer, d.internal_pointer, p.pert.internal_pointer, 1)
    forward!(m, d)
    return p
end

function forwardstep!(p::PhysicsState)
    m, d = p.model, p.data
    fill!(d.xfrc_applied, 0) # TODO: Check that fill!() is ok
    LibMuJoCo.mjv_applyPerturbPose(m.internal_pointer, d.internal_pointer, p.pert.internal_pointer, 0)
    LibMuJoCo.mjv_applyPerturbForce(m.internal_pointer, d.internal_pointer, p.pert.internal_pointer)
    step!(m, d)
    return p
end

####
#### EngineMode
####

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
# reset!(p, ::EngineMode) = (reset!(p.model); p)        # TODO: Are we implementing reset?
pausestep!(p, ::EngineMode) = pausestep!(p)
# prepare!(ui, p, ::EngineMode) = ui                    # TODO: Do we want this?
modeinfo(io1, io2, ui, p, ::EngineMode) = nothing
handlers(ui, p, ::EngineMode) = EventHandler[]


####
#### PassiveDynamics
####

struct PassiveDynamics <: EngineMode end
forwardstep!(p::PhysicsState, ::PassiveDynamics) = forwardstep!(p)


####
#### Controller
####

mutable struct Controller{F} <: EngineMode
    controller::F
    realtimefactor::Float64
end
Controller(controller) = Controller(controller, 1.0)

# TODO: This will destroy stateful controllers! Find a better way. Can we just ignore it completely? If so, delete it.
# function setup!(ui::UIState, p::PhysicsState, x::Controller)
#     dt = @elapsed x.controller(p.model, p.data)
#     x.realtimefactor = timestep(p.model.opt.timestep) / dt
#     return ui
# end

# TODO: Check that fill!() is ok
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

# TODO: Implement Trajectory mode later

####
#### Trajectory
####

# mutable struct Trajectory{TR<:AbsVec{<:AbsMat}} <: EngineMode
#     trajectories::TR
#     k::Int
#     t::Int

#     burstmode::Bool
#     bf_idx::Int
#     bf_range::LinRange{Float64}
#     bg_idx::Int
#     bg_range::LinRange{Float64}
#     doppler::Bool
#     function Trajectory{TR}(trajectories) where {TR<:AbsVec{<:AbsMat}}
#         new{TR}(trajectories, 1, 1, false, 1, LinRange(0, 1, 2), 1, LinRange(0, 1, 2), false)
#     end
# end
# Trajectory(trajectories::AbsVec{<:AbsMat}) = Trajectory{typeof(trajectories)}(trajectories)
# Trajectory(trajectories::AbsMat) = Trajectory([trajectories])


# function setup!(ui::UIState, p::PhysicsState, m::Trajectory)
#     setburstmodeparams!(m, p)
#     setstate!(m, p)
#     return ui
# end

# reset!(p::PhysicsState, m::Trajectory) = (m.t = 1; setstate!(m, p); p)

# function forwardstep!(p::PhysicsState, m::Trajectory)
#     m.t = inc(m.t, 1, getT(m))
#     setstate!(m, p)
#     return p
# end

# supportsreverse(::Trajectory) = true
# function reversestep!(p::PhysicsState, m::Trajectory)
#     m.t = dec(m.t, 1, getT(m))
#     setstate!(m, p)
#     return p
# end

# function prepare!(ui::UIState, p::PhysicsState, m::Trajectory)
#     if m.burstmode
#         bf = round(Int, m.bf_range[m.bf_idx])
#         bg = m.bg_range[m.bg_idx]
#         burst!(ui, p, gettraj(m), bf, m.t, gamma = bg, doppler = m.doppler)
#     end
#     return ui
# end

# function modeinfo(io1, io2, ui::UIState, p::PhysicsState, m::Trajectory)
#     println(io1, "t")
#     println(io2, "$(m.t)/$(getT(m))")
#     println(io1, "k")
#     println(io2, "$(m.k)/$(length(m.trajectories))")

#     n = length(m.bf_range)
#     println(io1, "Burst Mode")
#     println(io2, m.burstmode ? "Enabled" : "Disabled")
#     println(io1, "Burst Factor")
#     println(io2, "$(m.bf_idx)/$n")
#     println(io1, "Burst Gamma")
#     println(io2, "$(m.bg_idx)/$n")
#     println(io1, "Burst Doppler")
#     println(io2, m.doppler ? "Enabled" : "Disabled")

#     return nothing
# end

# function handlers(ui::UIState, p::PhysicsState, m::Trajectory)
#     return let ui=ui, p=p, m=m
#         [
#             onscroll(MOD_CONTROL, what = "Change burst factor") do s, ev
#                 m.bf_idx = clamp(m.bf_idx + ev.dy, 1, length(m.bf_range))
#             end,

#             onscroll(MOD_SHIFT, what = "Change burst decay rate") do s, ev
#                 m.bg_idx = clamp(m.bg_idx + ev.dy, 1, length(m.bg_range))
#             end,

#             onkey(GLFW.KEY_B, MOD_CONTROL, what = "Toggle burst mode") do s, ev
#                 ispress_or_repeat(ev.action) && (m.burstmode = !m.burstmode)
#             end,

#             onkey(GLFW.KEY_D, MOD_CONTROL, what = "Toggle burst mode doppler effect") do s, ev
#                 ispress_or_repeat(ev.action) && (m.doppler = !m.doppler)
#             end,

#             onkey(GLFW.KEY_UP, what = "Cycle forwards through trajectories") do s, ev
#                 if ispress_or_repeat(ev.action)
#                     m.k = inc(m.k, 1, length(m.trajectories))
#                     ax = axes(gettraj(m), 2)
#                     checkbounds(Bool, ax, m.t) || (m.t = last(ax))
#                     setburstmodeparams!(m, p)
#                     setstate!(m, p)
#                 end
#             end,

#             onkey(GLFW.KEY_DOWN, what = "Cycle backwards through trajectories") do s, ev
#                 if ispress_or_repeat(ev.action)
#                     m.k = dec(m.k, 1, length(m.trajectories))
#                     ax = axes(gettraj(m), 2)
#                     checkbounds(Bool, ax, m.t) || (m.t = last(ax))
#                     setburstmodeparams!(m, p)
#                     setstate!(m, p)
#                 end
#             end,
#         ]
#     end
# end

# function setburstmodeparams!(m::Trajectory, p::PhysicsState)
#     steps = 40 # n scroll increments

#     # Params calibrated on cartpole and scaled accordingly
#     # Rendering up to bf0_max states for a len0 length trajectory
#     # with a timestep of dt0 and gamma0 looks reasonalbe.
#     len0 = 100
#     bf0_max = 50
#     gamma0 = 0.85
#     dt0 = 0.01

#     dt = timestep(p.model)
#     len = size(gettraj(m), 2)

#     bf_max = round(Int, bf0_max * (len / len0) * (dt / dt0))
#     gamma = gamma0^(dt / dt0)

#     m.bf_range = LinRange{Float64}(1, bf_max, steps) # render between 1 and bf_max states
#     m.bg_range = LinRange{Float64}(gamma, 1, steps)

#     m.bf_idx = round(Int, steps / 2)
#     m.bg_idx = steps

#     return m
# end

# @inline function setstate!(m::Trajectory, p::PhysicsState)
#     LyceumMuJoCo.setstate!(p.model, view(gettraj(m), :, m.t))
#     return m
# end

# getT(m::Trajectory) = size(m.trajectories[m.k], 2)
# gettraj(m::Trajectory) = m.trajectories[m.k]


####
#### Util
####

# function burst!(
#     ui::UIState,
#     p::PhysicsState,
#     states::AbstractMatrix,
#     n::Integer,
#     t::Integer;
#     gamma::Real = 0.9995,
#     alphamin::Real = 0.05,
#     alphamax::Real = 0.55,
#     doppler::Bool = true,
# )
#     T = size(states, 2)

#     T >= n > 0 || error("n must be in range [1, size(states, 2)]")
#     0 < t || error("t must be > 0")
#     0 < gamma <= 1|| error("gamma must be in range (0, 1)")
#     0 < alphamin <= 1 || error("alphamin must be in range (0, 1]")
#     0 < alphamax <= 1 || error("alphamin must be in range (0, 1]")

#     scn = ui.scn
#     sim = getsim(p.model)
#     n = min(n, fld(MAXGEOM, sim.m.ngeom))
#     geoms = unsafe_wrap(Array, scn[].geoms, scn[].maxgeom)

#     function color!(tprime, from)
#         for i = from:scn[].ngeom
#             geom = @inbounds geoms[i]
#             if geom.category == Int(MJCore.mjCAT_DYNAMIC)
#                 dist = abs(tprime - t)
#                 r, g, b, alpha0 = geom.rgba

#                 alpha = alpha0 * gamma^dist

#                 if doppler
#                     g = 0
#                     if tprime < t
#                         beta = (dist + 1) / t / 2 + 0.5
#                         r = beta * r
#                         b = (1 - beta) * b
#                     else
#                         beta = dist / (T - t) / 2 + 0.5
#                         r = (1 - beta) * r
#                         b = beta * b
#                     end
#                     r = clamp(r, 0, 1)
#                     b = clamp(b, 0, 1)
#                 end

#                 geoms[i] = @set!! geom.rgba = SVector{4,Cfloat}(r, g, b, alpha)
#             end
#         end
#     end

#     LyceumMuJoCo.setstate!(p.model, view(states, :, t))
#     mjv_updateScene(
#         sim.m,
#         sim.d,
#         ui.vopt,
#         p.pert,
#         ui.cam,
#         MJCore.mjCAT_ALL,
#         scn,
#     )

#     if n > 1
#         fromidx = scn[].ngeom + 1
#         from = scn[].ngeom + 1
#         for tprime in Iterators.map(x -> round(Int, x), LinRange(1, T, n))
#             if tprime != t
#                 LyceumMuJoCo.setstate!(p.model, view(states, :, tprime))
#                 mjv_addGeoms(
#                     sim.m,
#                     sim.d,
#                     ui.vopt,
#                     p.pert,
#                     MJCore.mjCAT_DYNAMIC,
#                     scn,
#                 )
#                 color!(tprime, from)
#             end
#             from = scn[].ngeom + 1
#         end
#     end

#     # reset the model to the state it had when it was passed in
#     LyceumMuJoCo.setstate!(p.model, view(states, :, t))

#     return ui
# end
