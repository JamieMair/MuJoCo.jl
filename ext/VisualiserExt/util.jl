# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

const ASCII = raw"""
 __  __            _        _____       _ _ 
|  \/  |          | |      / ____|     (_) |
| \  / |_   _     | | ___ | |     ___   _| |
| |\/| | | | |_   | |/ _ \| |    / _ \ | | |
| |  | | |_| | |__| | (_) | |___| (_) || | |
|_|  |_|\__,_|\____/ \___/ \_____\___(_) |_|
                                      _/ |  
                                     |__/   
"""

function set_string!(buffer::AbstractArray, s::String)
    buffer_size = length(buffer)
    s_length = length(s)
    @assert s_length < buffer_size "The buffer for this string is only $buffer_size large, but tried to put in a string of length $s_length"
    buffer[1:s_length] = codeunits(s)
    buffer[s_length+1] = zero(eltype(buffer)) # Assume zero-terminated string
    nothing
end

function init_figsensor!(figsensor::VisualiserFigure)
    figsensor.flg_extend = 1
    figsensor.flg_barplot = 1
    set_string!(figsensor.title, "Sensor Data")
    set_string!(figsensor.title, "%.0f")
    figsensor.gridsize .= @SVector [2, 3]
    figsensor.range .= @SMatrix [0;1;;-1;1]
    return figsensor
end

# Useful for trajectory mode
@inline inc(x::Integer, min::Integer, max::Integer) = ifelse(x == max, min, x + 1)
@inline dec(x::Integer, min::Integer, max::Integer) = ifelse(x == min, max, x - 1)

function startffmpeg(w::Integer, h::Integer, rout::Integer; squashoutput::Bool = true)
    w > 0 && h > 0 || error("w and h must be > 0")

    dst = tempname(pwd()) * ".mp4"
    arg = `-y -f rawvideo -pixel_format rgb24 -video_size $(w)x$(h) -use_wallclock_as_timestamps true -i pipe:0 -c:v libx264 -preset ultrafast -tune animation -vf "vflip" -r $rout $dst`

    FFMPEG.@ffmpeg_env begin
        indata = Base.PipeEndpoint()
        if squashoutput
            p = Base._spawn(`$(FFMPEG.ffmpeg) $arg`, Base.SpawnIO[indata, devnull, devnull])
        else
            p = Base._spawn(`$(FFMPEG.ffmpeg) $arg`, Base.SpawnIO[indata, stdout, stderr])
        end
        p.in = indata
        return p, dst
    end
end

function safe_unlock(lck::ReentrantLock)
    if islocked(lck) && current_task() === lck.locked_by
        unlock(lck)
    end
    return nothing
end

"""
    spinwait(delay)

Spin in a tight loop for at least `delay` seconds.

Note this function is only accurate on the order of approximately `@elapsed time()`
seconds.
"""
@inline function spinwait(dt::Real)
    dt > 0 || error("dt must be > 0")
    t0 = time()
    while time() - t0 < dt end
    return nothing
end

@inline function str2unicode(s::AbstractString)
    length(s) == 1 || error("s must be a single length string")
    return Int(first(s))
end

# From PrettyTables.jl: https://github.com/ronisbr/PrettyTables.jl/blob/d8a948a2dd14df66852c9702387c13f4dfce30d8/src/backends/text/print.jl#L50
function get_terminalsize()
    return withenv("LINES" => -1, "COLUMNS" => -1) do
        displaysize(stdout)
    end
end
