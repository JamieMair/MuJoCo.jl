const ASCII = raw"""
  _
 | |
 | |    _   _  ___ ___ _   _ _ __ ___
 | |   | | | |/ __/ _ \ | | | '_ ` _ \
 | |___| |_| | (_|  __/ |_| | | | | | |
 |______\__, |\___\___|\__,_|_| |_| |_|
         __/ |
        |___/
"""

function str2vec(s::String, len::Int)
    str = zeros(UInt8, len)
    str[1:length(s)] = codeunits(s)
    return str
end

function init_figsensor!(figsensor::Ref{mjvFigure})
    figsensor[].flg_extend = 1
    figsensor[].flg_barplot = 1
    figsensor[].title = str2vec("Sensor Data", length(figsensor[].title))
    figsensor[].yformat = str2vec("%.0f", length(figsensor[].yformat))
    figsensor[].gridsize = [2, 3]
    figsensor[].range = [[0 1], [-1 1]]
    return figsensor
end

@inline inc(x::Integer, min::Integer, max::Integer) = ifelse(x == max, min, x + 1)
@inline dec(x::Integer, min::Integer, max::Integer) = ifelse(x == min, max, x - 1)

function startffmpeg(w::Integer, h::Integer, rout::Integer; squashoutput::Bool = true)
    w > 0 && h > 0 || error("w and h must be > 0")

    dst = tempname() * ".mp4"
    arg = `-y -f rawvideo -pixel_format rgb24 -video_size $(w)x$(h) -use_wallclock_as_timestamps true -i pipe:0 -c:v libx264 -preset ultrafast -tune animation -vf "vflip" -r $rout $dst`

    FFMPEG.@ffmpeg_env begin
        in = Base.PipeEndpoint()
        if squashoutput
            p = Base._spawn(`$(FFMPEG.ffmpeg) $arg`, Any[in, devnull, devnull])
        else
            p = Base._spawn(`$(FFMPEG.ffmpeg) $arg`, Any[in, stdout, stderr])
        end
        p.in = in
        return p, dst
    end
end

function safe_unlock(lck::ReentrantLock)
    if islocked(lck) && current_task() === lck.locked_by
        unlock(lck)
    end
    return
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