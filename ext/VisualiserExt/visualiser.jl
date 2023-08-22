# Most this file is slightly modified from https://github.com/Lyceum/LyceumMuJoCoViz.jl

import Base: @lock, @lock_nofail

if isdefined(Base, :get_extension)
    import MuJoCo.LibMuJoCo: Model, Data
    import MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
    import MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render
    using MuJoCo.Visualiser
    using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
    using Observables: AbstractObservable, Observable, on, off
else
    import ..MuJoCo.LibMuJoCo: Model, Data
    import ..MuJoCo.LibMuJoCo: mjvScene, mjvCamera, mjvOption, mjvFigure
    import ..MuJoCo.LibMuJoCo: mjrContext, mjrRect, mjr_render
    using ..MuJoCo.Visualiser
    using ..GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
    using ..Observables: AbstractObservable, Observable, on, off
end

const Maybe{T} = Union{T, Nothing}  # From LyceumBase.jl
const MAXGEOM = 10000               # preallocated geom array in mjvScene
const MIN_REFRESHRATE = 30          # minimum rate when sim can't run at native refresh rate
const RNDGAMMA = 0.9

const RES_HD = (1280, 720)
const RES_FHD = (1920, 1080)
const RES_XGA = (1024, 768)
const RES_SXGA = (1280, 1024)

include("util.jl")
include("glfw.jl")
include("ratetimer.jl")
include("types.jl")
include("functions.jl")
include("modes.jl")
include("defaulthandlers.jl")


# ----------------------------------------------------------------------------------

"""
Add docstring here
"""
function visualise(m::Model, d::Data; controller=nothing)
    modes = EngineMode[PassiveDynamics()]
    !isnothing(controller) && push!(modes, Controller(controller))
    e = Engine(default_windowsize(), m, d, Tuple(modes))
    run(e)
end

"""
Run the visualiser engine
"""
function run(e::Engine)

    # Render the first frame before opening window
    prepare!(e)
    e.ui.refreshrate = GetRefreshRate()
    e.ui.lastrender = time()
    GLFW.ShowWindow(e.manager.state.window)

    # Run the simulation/mode in a different thread
    modetask = Threads.@spawn runphysics(e)

    # Print help info
    println(ASCII)
    println("Press \"F1\" to show the help message.")

    # Run the visuals
    runui(e)
    wait(modetask)
    return nothing
end

"""
Run the UI
"""
function runui(e::Engine)
    shouldexit = false
    trecord = 0.0
    try
        while !shouldexit

            # Check for window interaction and prepare visualisation
            @lock e.phys.lock begin
                GLFW.PollEvents()
                prepare!(e)
            end

            # Render
            render(e)
            trender = time()

            # Match the rendering rate to desired rates
            rt = 1 / (trender - e.ui.lastrender)
            @lock e.ui.lock begin
                e.ui.refreshrate = RNDGAMMA * e.ui.refreshrate + (1 - RNDGAMMA) * rt
                e.ui.lastrender = trender
                shouldexit = e.ui.shouldexit | GLFW.WindowShouldClose(e.manager.state.window)
            end

            # Handle frame recording
            tnow = time()
            if e.ffmpeghandle !== nothing && tnow - trecord > 1 / e.min_refreshrate
                trecord = tnow
                recordframe(e)
            end
            yield() # Visualisation should give way to running the physics model
        end
    finally
        @lock e.ui.lock begin
            e.ui.shouldexit = true
        end
        GLFW.DestroyWindow(e.manager.state.window)
    end
    return
end

"""
Prepare the visualisation engine for rendering
"""
function prepare!(e::Engine)
    ui, p = e.ui, e.phys
    m, d = p.model, p.data
    LibMuJoCo.mjv_updateScene(
        m.internal_pointer, 
        d.internal_pointer, 
        ui.vopt.internal_pointer, 
        p.pert.internal_pointer, 
        ui.cam.internal_pointer, 
        LibMuJoCo.mjCAT_ALL,
        ui.scn.internal_pointer
    )
    prepare!(ui, p, mode(e))
    return e
end

"""
Render a frame
"""
function render(e::Engine)
    w, h = GLFW.GetFramebufferSize(e.manager.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    mjr_render(rect, e.ui.scn.internal_pointer, e.ui.con.internal_pointer)
    e.ui.showinfo && overlay_info(rect, e)
    GLFW.SwapBuffers(e.manager.state.window)
    return nothing
end

"""
Run the MuJoCo model.

This function handles simulating the model in pause, forward, and reverse mode.
Note that reverse mode is only implemented for the `Trajectory` EngineMode.
"""
function runphysics(e::Engine)
    p = e.phys
    ui = e.ui
    resettime!(p) # reset sim and world clocks to 0

    try
        while true
            shouldexit, lastrender, reversed, paused, refrate, = @lock_nofail ui.lock begin
                ui.shouldexit, ui.lastrender, ui.reversed, ui.paused, ui.refreshrate
            end

            if shouldexit
                break
            elseif (time() - lastrender) > 1 / e.min_refreshrate
                yield()
                continue
            else
                @lock p.lock begin
                    elapsedworld = time(p.timer)

                    # advance sim
                    if ui.paused
                        pausestep!(p, mode(e))
                    elseif ui.reversed && p.elapsedsim > elapsedworld
                        reversestep!(p, mode(e))
                        p.elapsedsim -= timestep(p.model)
                    elseif !ui.reversed && p.elapsedsim < elapsedworld
                        forwardstep!(p, mode(e))
                        p.elapsedsim += timestep(p.model)
                    end
                end
            end
        end
    finally
        @lock ui.lock begin
            ui.shouldexit = true
        end
    end
    return nothing
end
