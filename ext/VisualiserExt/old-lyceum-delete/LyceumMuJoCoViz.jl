# module LyceumMuJoCoViz

# using Base: RefValue, @lock, @lock_nofail

# Stdlib
# using Printf: @printf

# 3rd party
# using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
# using PrettyTables: pretty_table
# using BangBang: @set!!
# using StaticArrays: SVector, MVector
# using DocStringExtensions
# using Observables: AbstractObservable, Observable, on, off
# using FFMPEG

# # Lyceum
# using MuJoCo, MuJoCo.MJCore
# using LyceumMuJoCo
# import LyceumMuJoCo: reset!
# using LyceumBase: LyceumBase, Maybe, AbsVec, AbsMat


# export visualize


# const FONTSCALE = MJCore.FONTSCALE_150 # can be 100, 150, 200
# const MAXGEOM = 10000 # preallocated geom array in mjvScene
# const MIN_REFRESHRATE = 30 # minimum rate when sim cannot run at the native refresh rate
# const SIMGAMMA = 0.99
# const RNDGAMMA = 0.9
# const VIDFPS = 40

# const RES_HD = (1280, 720)
# const RES_FHD = (1920, 1080)
# const RES_XGA = (1024, 768)
# const RES_SXGA = (1280, 1024)


# include("util.jl")
# include("glfw.jl")
# include("ratetimer.jl")
# include("types.jl")
# include("functions.jl")
# include("modes.jl")
# include("defaulthandlers.jl")


# function __init__()
#     if Threads.nthreads() == 1
#         @warn "LyceumMuJoCoViz is designed to run multi-threaded, but the current Julia session was started with only one thread. Degraded performance will occur. To enable multi-threading, set JULIA_NUM_THREADS to a value greater than 1 before starting Julia."
#     end
#     return
# end


"""
    $(TYPEDSIGNATURES)

Starts an interactive visualization of `model`, which can be a valid subtype of
`AbstractMuJoCoEnvironment`, a `MJSim` simulation, or the path to a valid MuJoCo model file.
The visualizer has several "modes" that allow you to visualize passive dynamics,
play back recorded trajectories, and run a controller interactively. The passive dynamics
mode depends only on `model` and is always available, while the other modes are specified
by the keyword arguments below.

# Keywords

- `trajectories`: a single trajectory or vector of trajectories, where each
    trajectory is an AbstractMatrix of states with size `(length(statespace(model)), T)` and
    `T` is the length of the trajectory. Note that each trajectory can have different length.
- `controller`: a callback function with the signature `controller(model)`, called at each
    timestep, that applies a control input to the system.
- `windowsize`: resolution of visualizer window. Defaults to an internal heuristic.

# Examples
```julia
using LyceumMuJoCo, LyceumMuJoCoViz
env = LyceumMuJoCo.HopperV2()
T = 100
states = Array(undef, statespace(env), T)
for t = 1:T
    step!(env)
    states[:, t] .= getstate(env)
end
visualize(
    env,
    trajectories = states,
    controller = env -> setaction!(env, rand(actionspace(env)))
)
```
"""
# function visualize(
#     model::Union{AbstractString,MJSim,AbstractMuJoCoEnvironment};
#     trajectories::Union{Nothing,AbsMat,AbsVec{<:AbsMat}} = nothing,
#     controller = nothing,
#     windowsize::NTuple{2,Integer} = default_windowsize()
# )
#     model isa AbstractString && (model = MJSim(model))
#     reset!(model)

#     modes = EngineMode[PassiveDynamics()]
#     !isnothing(trajectories) && push!(modes, Trajectory(trajectories))
#     !isnothing(controller) && push!(modes, Controller(controller))

#     run(Engine(windowsize, model, Tuple(modes)))
#     return
# end


# function run(e::Engine)
#     if e.phys.model isa AbstractMuJoCoEnvironment
#         e.ui.reward = getreward(e.phys.model)
#     end

#     # render first frame before opening window
#     prepare!(e)
#     render(e)
#     e.ui.refreshrate = GetRefreshRate()
#     e.ui.lastrender = time()
#     GLFW.ShowWindow(e.mngr.state.window)

#     # run the simulation/mode in second thread
#     modetask = Threads.@spawn runphysics(e)

#     println(ASCII)
#     println("Press \"F1\" to show the help message.")

#     runui(e)
#     wait(modetask)
#     return
# end


# function runui(e::Engine)
#     shouldexit = false
#     trecord = 0.0
#     try
#         while !shouldexit
#             @lock e.phys.lock begin
#                 GLFW.PollEvents()
#                 prepare!(e)
#             end

#             render(e)
#             trender = time()

#             rt = 1 / (trender - e.ui.lastrender)
#             @lock e.ui.lock begin
#                 e.ui.refreshrate = RNDGAMMA * e.ui.refreshrate + (1 - RNDGAMMA) * rt
#                 e.ui.lastrender = trender
#                 shouldexit = e.ui.shouldexit |= GLFW.WindowShouldClose(e.mngr.state.window)
#             end

#             tnow = time()
#             if e.ffmpeghandle !== nothing && tnow - trecord > 1 / e.min_refreshrate
#                 trecord = tnow
#                 recordframe(e)
#             end

#             yield()
#         end
#     finally
#         @lock e.ui.lock begin
#             e.ui.shouldexit = true
#         end
#         GLFW.DestroyWindow(e.mngr.state.window)
#     end
#     return
# end

# function prepare!(e::Engine)
#     ui, p = e.ui, e.phys
#     sim = getsim(p.model)
#     p.model isa AbstractMuJoCoEnvironment && (ui.reward = getreward(p.model))
#     mjv_updateScene(
#         sim.m,
#         sim.d,
#         ui.vopt,
#         p.pert,
#         ui.cam,
#         MJCore.mjCAT_ALL,
#         ui.scn,
#     )
#     prepare!(ui, p, mode(e))
#     return e
# end

# function render(e::Engine)
#     w, h = GLFW.GetFramebufferSize(e.mngr.state.window)
#     rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
#     mjr_render(rect, e.ui.scn, e.ui.con)
#     e.ui.showinfo && overlay_info(rect, e)
#     GLFW.SwapBuffers(e.mngr.state.window)
#     return
# end


# function runphysics(e::Engine)
#     p = e.phys
#     ui = e.ui
#     resettime!(p) # reset sim and world clocks to 0

#     try
#         while true
#             shouldexit, lastrender, reversed, paused, refrate, = @lock_nofail ui.lock begin
#                 ui.shouldexit, ui.lastrender, ui.reversed, ui.paused, ui.refreshrate
#             end

#             if shouldexit
#                 break
#             elseif (time() - lastrender) > 1 / e.min_refreshrate
#                 yield()
#                 continue
#             else
#                 @lock p.lock begin
#                     elapsedworld = time(p.timer)

#                     # advance sim
#                     if ui.paused
#                         pausestep!(p, mode(e))
#                     elseif ui.reversed && p.elapsedsim > elapsedworld
#                         reversestep!(p, mode(e))
#                         p.elapsedsim -= timestep(p.model)
#                     elseif !ui.reversed && p.elapsedsim < elapsedworld
#                         forwardstep!(p, mode(e))
#                         p.elapsedsim += timestep(p.model)
#                     end
#                 end
#             end
#         end
#     finally
#         @lock ui.lock begin
#             ui.shouldexit = true
#         end
#     end

#     return
# end

# end # module
