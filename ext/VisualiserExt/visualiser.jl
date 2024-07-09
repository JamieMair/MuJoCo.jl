# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

function MuJoCo.Visualiser.visualise!(
    m::Model, d::Data; 
    controller = nothing, 
    trajectories = nothing,
    resolution::Union{Tuple{Integer, Integer}, Nothing} = nothing
)
    modes = EngineMode[]
    !isnothing(controller) && push!(modes, Controller(controller))
    !isnothing(trajectories) && push!(modes, Trajectory(trajectories))
    push!(modes, PassiveDynamics())

    window_size = if isnothing(resolution)
        default_windowsize()
    else
        @assert resolution[1] > 0 "Width of resolution should be greater than 0."
        @assert resolution[2] > 0 "Height of resolution should be greater than 0."
        resolution
    end

    run!(Engine(window_size, m, d, Tuple(modes)))
    return nothing
end

"""
Run the visualiser engine
"""
function run!(e::Engine)

    # Render the first frame before opening window
    prepare!(e)
    e.ui.refreshrate = GetRefreshRate()
    e.ui.lastrender = time()
    GLFW.ShowWindow(e.manager.state.window)

    # Run the simulation/mode in a different thread
    modetask = Threads.@spawn runphysics!(e)

    # Print help info
    println(ASCII)
    println("Press \"F1\" to show the help message.")

    # Run the visuals
    runui!(e)
    wait(modetask)
    return nothing
end

"""
Run the UI
"""
function runui!(e::Engine)
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
            render!(e)
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
        m, 
        d, 
        ui.vopt, 
        p.pert, 
        ui.cam, 
        LibMuJoCo.mjCAT_ALL,
        ui.scn
    )
    prepare!(ui, p, mode(e))
    return e
end

"""
Render a frame
"""
function render!(e::Engine)
    w, h = GLFW.GetFramebufferSize(e.manager.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    LibMuJoCo.mjr_render(rect, e.ui.scn, e.ui.con)
    e.ui.showinfo && overlay_info(rect, e)
    GLFW.SwapBuffers(e.manager.state.window)
    return nothing
end

"""
Run the MuJoCo model.

This function handles simulating the model in pause, forward, and reverse mode.
Note that reverse mode is only implemented for the `Trajectory` EngineMode.
"""
function runphysics!(e::Engine)
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
