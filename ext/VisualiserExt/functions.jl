# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

# Anything commented out is a function we have copied but not yet changed. Some of these will not be required in our final version and can be deleted.

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

    writedescription(io, e.handlers)
    handlerdescription = String(take!(io))

    writedescription(io, e.modehandlers)
    modehandlerdescription = String(take!(io))

    println("Standard Commands:")
    print(handlerdescription)
    if !isempty(modehandlerdescription)
        println("$(nameof(mode(e))) Mode Commands:")
        print(modehandlerdescription)
    end
    println()
    println()

    return
end

function writedescription(io, hs::Vector{EventHandler})
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

    return
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

    # TODO: Need to get mjFRAMESTRING
    # println(io1, "Frame")
    # println(io2, MJCore.mjFRAMESTRING[e.ui.vopt[].frame+1])

    # println(io1, "Label")
    # println(io2, MJCore.mjLABELSTRING[e.ui.vopt[].label+1])

    # mode specific info
    println(io1, "Mode Info")
    println(io2)
    modeinfo(io1, io2, ui, phys, mode(e))

    info1 = string(chomp(String(take!(io1))))
    info2 = string(chomp(String(take!(io2))))

    LibMuJoCo.mjr_overlay(
        LibMuJoCo.mjtFont(0),    # mjFONT_NORMAL
        LibMuJoCo.mjtGridPos(2), # mjGRID_BOTTOMRIGHT
        rect,
        info1,
        info2,
        ui.con.internal_pointer,
    )

    return
end


# function startrecord!(e::Engine)
#     window = e.mngr.state.window
#     SetWindowAttrib(window, GLFW.RESIZABLE, 0)
#     w, h = GLFW.GetFramebufferSize(window)
#     resize!(e.framebuf, 3 * w * h)
#     e.ffmpeghandle, e.videodst = startffmpeg(w, h, e.min_refreshrate)
#     @info "Recording video. Window resizing temporarily disabled"
#     return e
# end

# function recordframe(e::Engine)
#     w, h = GLFW.GetFramebufferSize(e.mngr.state.window)
#     rect = mjrRect(Cint(0), Cint(0), Cint(w), Cint(h))
#     mjr_readPixels(e.framebuf, C_NULL, rect, e.ui.con)
#     write(e.ffmpeghandle, e.framebuf)
#     return nothing
# end

# function stoprecord!(e::Engine)
#     SetWindowAttrib(e.mngr.state.window, GLFW.RESIZABLE, 1)
#     @info "Recording finished, window resizing re-enabled. Waiting for transcoding to finish."
#     ispaused = e.ui.paused
#     setpause!(e.ui, e.phys, true)
#     close(e.ffmpeghandle)
#     @info "Finished recording! Video saved to $(e.videodst)"
#     e.ffmpeghandle = e.videodst = nothing
#     setpause!(e.ui, e.phys, ispaused)
#     return e
# end

function setpause!(ui::UIState, p::PhysicsState, status::Bool)
    status ? stop!(p.timer) : start!(p.timer)
    ui.paused = status
    ui
end
