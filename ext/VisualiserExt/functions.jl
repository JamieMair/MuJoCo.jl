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
        custom_pretty_table(
            io, whens, whats,
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

"""
    custom_pretty_table(io, col1, col2; header=nothing, alignment=nothing, columns_width=nothing, kwargs...)

Custom implementation of pretty_table for backwards compatibility.
This function handles two-column tables with proper alignment and truncation.
Takes two vectors as input for the columns.
"""
function custom_pretty_table(io::IO, col1::AbstractVector, col2::AbstractVector; 
                            header=nothing, 
                            alignment=nothing, 
                            columns_width=nothing, 
                            linebreaks=false, 
                            autowrap=false, 
                            kwargs...)
    
    # Ensure both columns have the same length
    if length(col1) != length(col2)
        error("custom_pretty_table requires both column vectors to have the same length")
    end
    
    nrows = length(col1)
    
    # Set default column widths if not provided
    if columns_width === nothing
        _, terminal_cols = get_terminalsize()
        # Reserve space for separators and padding: " | " = 3 chars
        available_width = terminal_cols - 3
        columns_width = [div(available_width, 2), div(available_width, 2)]
    end
    
    w1, w2 = columns_width
    
    # Set default alignment if not provided
    if alignment === nothing
        alignment = [:l, :l]  # left align both columns by default
    end
    
    # Helper function to truncate text to fit width
    function truncate_text(text::AbstractString, width::Int)
        if length(text) <= width
            return text
        else
            return text[1:max(1, width-3)] * "..."
        end
    end
    
    # Helper function to wrap text
    function wrap_text(text::AbstractString, width::Int)
        if !autowrap || length(text) <= width
            return [truncate_text(text, width)]
        end
        
        lines = String[]
        remaining = string(text)
        while length(remaining) > width
            # Find a good break point (space or punctuation)
            break_point = width
            for i in min(width, length(remaining)):-1:max(1, width÷2)
                if remaining[i] in [' ', '\t', '-', ',', '.', ';', ':', '!', '?']
                    break_point = i
                    break
                end
            end
            
            push!(lines, truncate_text(remaining[1:break_point], width))
            remaining = lstrip(remaining[break_point+1:end])
        end
        if !isempty(remaining)
            push!(lines, truncate_text(remaining, width))
        end
        return lines
    end
    
    # Helper function to align text within a given width
    function align_text(text::AbstractString, width::Int, align::Symbol)
        text = truncate_text(text, width)
        if align == :c || align == :center
            padding = width - length(text)
            left_pad = div(padding, 2)
            right_pad = padding - left_pad
            return " "^left_pad * text * " "^right_pad
        elseif align == :r || align == :right
            return lpad(text, width)
        else  # :l or :left (default)
            return rpad(text, width)
        end
    end
    
    # Print header if provided
    if header !== nothing && length(header) >= 2
        col1_header = align_text(string(header[1]), w1, alignment[1])
        col2_header = align_text(string(header[2]), w2, alignment[2])
        println(io, col1_header * " | " * col2_header)
        
        # Print separator line
        sep1 = "-"^w1
        sep2 = "-"^w2
        println(io, sep1 * "-+-" * sep2)
    end
    
    # Print data rows
    for i in 1:nrows
        col1_text = string(col1[i])
        col2_text = string(col2[i])
        
        # Handle text wrapping
        col1_lines = wrap_text(col1_text, w1)
        col2_lines = wrap_text(col2_text, w2)
        
        # Determine the number of lines needed for this row
        max_lines = max(length(col1_lines), length(col2_lines))
        
        # Print each line of the row
        for line_idx in 1:max_lines
            col1_line = line_idx <= length(col1_lines) ? col1_lines[line_idx] : ""
            col2_line = line_idx <= length(col2_lines) ? col2_lines[line_idx] : ""
            
            col1_aligned = align_text(col1_line, w1, alignment[1])
            col2_aligned = align_text(col2_line, w2, alignment[2])
            
            println(io, col1_aligned * " | " * col2_aligned)
        end
    end
    
    return nothing
end
