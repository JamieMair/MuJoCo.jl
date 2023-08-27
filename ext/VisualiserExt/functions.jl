# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

####
#### PhysicsState
####

resettime!(phys::PhysicsState) = (reset!(phys.timer); phys.elapsedsim = 0; phys)


####
#### UIState
####

function alignscale!(ui::UIState, m::Model)
    ui.cam.lookat .= m.stat.center
    ui.cam.distance = 1.5 * m.stat.extent
    ui.cam.type = LibMuJoCo.mjCAMERA_FREE
    return ui
end

####
#### Engine
####

@inline mode(e::Engine, idx::Integer = e.curmodeidx) = e.modes[idx]

function switchmode!(e::Engine, idx::Integer)
    teardown!(e.ui, e.phys, mode(e))
    deregister!(e.manager, e.modehandlers...)

    e.curmodeidx = idx
    e.modehandlers = handlers(e.ui, e.phys, mode(e))
    setup!(e.ui, e.phys, mode(e))
    register!(e.manager, e.modehandlers...)

    return e
end

function printhelp(e::Engine)
    io = e.ui.io1

    writedescription!(io, getmousedescription())
    mousedescription = String(take!(io))

    writedescription!(io, e.handlers)
    handlerdescription = String(take!(io))

    writedescription!(io, e.modehandlers)
    modehandlerdescription = String(take!(io))

    println("Mouse click commands:")
    println(mousedescription)

    println("Standard button commands:")
    println(handlerdescription)

    if !isempty(modehandlerdescription)
        println("$(nameof(mode(e))) mode commands:")
        print(modehandlerdescription)
    end
    println()
    println()

    return
end

function getmousedescription()
    return [
        (when="Left click", what="Rotate scene"),
        (when="Right click", what="Translate scene"),
        (when="Left double-click", what="Select body"),
        (when="Right double-click", what="Point camera at body"),
        (when="Left+CTRL", what="Rotate selected body"),
        (when="Right+CTRL", what="Translate selected body"),
        (when="Right double-click + CTRL", what="Fix camera on selected body"),
    ]
end

function writedescription!(io, hs::Union{Vector{EventHandler},Vector{<:NamedTuple}})
    if !isempty(hs)
        whens = String[]
        whats = String[]
        for h in hs
            if h.when !== nothing && h.what !== nothing
                push!(whens, h.when)
                push!(whats, h.what)
            elseif h.what !== nothing
                push!(whens, "----")
                push!(whats, h.what)
            end
        end

        header = ["Command", "Description"]
        _, ncols = get_terminalsize()
        w1max = max(maximum(length, whens), length(first(header)))
        w2max = max(maximum(length, whats), length(first(header)))
        
        w1 = min(w1max, div(ncols, 2))
        w2 = min(w2max, ncols - w1 - 4 * length(header)) # each column is padded by 4 spaces
        pretty_table(
            io, hcat(whens, whats); 
            header=["Command", "Description"],
            alignment = [:c, :l],
            linebreaks = true, 
            autowrap = true,
            columns_width = [w1, w2]
        )
    end
    return nothing
end

function overlay_info(rect::mjrRect, e::Engine)
    ui = e.ui
    io1 = ui.io1
    io2 = ui.io2
    phys = e.phys
    data = phys.data

    seekstart(io1)
    seekstart(io2)

    println(io1, "Mode")
    println(io2, nameof(mode(e)))

    println(io1, "Status")
    if ui.paused
        println(io2, "Paused")
    elseif ui.reversed
        println(io2, "Reverse Simulation")
    else
        println(io2, "Forward Simulation")
    end

    println(io1, "Time")
    @printf io2 "%.3f s\n" data.time

    println(io1, "Refresh Rate")
    @printf io2 "%d Hz\n" ui.refreshrate

    println(io1, "Resolution")
    @printf io2 "(%d, %d)\n" e.manager.state.width e.manager.state.height

    println(io1, "Sim Speed")
    if ui.speedmode
        if ui.speedfactor < 1
            @printf io2 "%.5gx (slower)\n" 1 / ui.speedfactor
        else
            @printf io2 "%.5gx (faster)\n" ui.speedfactor
        end
    else
        println(io2, "1")
    end

    println(io1, "Frame")
    println(io2, mjFRAMESTRING[e.ui.vopt.frame+1])

    println(io1, "Label")
    println(io2, mjLABELSTRING[e.ui.vopt.label+1])

    # mode specific info
    println(io1, "Mode Info")
    println(io2)
    modeinfo(io1, io2, ui, phys, mode(e))

    info1 = string(chomp(String(take!(io1))))
    info2 = string(chomp(String(take!(io2))))

    LibMuJoCo.mjr_overlay(
        LibMuJoCo.mjFONT_NORMAL,
        LibMuJoCo.mjGRID_BOTTOMLEFT,
        rect,
        info1,
        info2,
        ui.con,
    )

    return nothing
end

function startrecord!(e::Engine)
    window = e.manager.state.window
    SetWindowAttrib(window, GLFW.RESIZABLE, 0)
    w, h = GLFW.GetFramebufferSize(window)
    resize!(e.framebuf, 3 * w * h)
    e.ffmpeghandle, e.videodst = startffmpeg(w, h, e.min_refreshrate)
    @info "Recording video. Window resizing temporarily disabled"
    return e
end

function recordframe(e::Engine)
    w, h = GLFW.GetFramebufferSize(e.manager.state.window)
    rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
    LibMuJoCo.mjr_readPixels(e.framebuf, C_NULL, rect, e.ui.con)
    write(e.ffmpeghandle, e.framebuf)
    return nothing
end

function stoprecord!(e::Engine)
    SetWindowAttrib(e.manager.state.window, GLFW.RESIZABLE, 1)
    @info "Recording finished, window resizing re-enabled. Waiting for transcoding to finish."
    ispaused = e.ui.paused
    setpause!(e.ui, e.phys, true)
    close(e.ffmpeghandle)
    @info "Finished recording! Video saved to $(e.videodst)"
    e.ffmpeghandle = e.videodst = nothing
    setpause!(e.ui, e.phys, ispaused)
    return e
end

function setpause!(ui::UIState, p::PhysicsState, status::Bool)
    status ? stop!(p.timer) : start!(p.timer)
    ui.paused = status
    return ui
end
