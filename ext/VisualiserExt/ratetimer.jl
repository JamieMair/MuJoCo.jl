# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

mutable struct RateTimer
    tlast::Float64
    elapsed::Float64
    rate::Float64
    paused::Bool
    tpaused::Float64
    function RateTimer(rate)
        t = time_ns()
        new(t, 0, rate, true, t)
    end
end
RateTimer() = RateTimer(1)

function Base.time_ns(rt::RateTimer)
    if !rt.paused
        tnow = time_ns()
        elapsed = tnow - rt.tlast
        rt.tlast = tnow
        rt.elapsed += elapsed * rt.rate
    end
    return rt.elapsed
end

Base.time(rt::RateTimer) = time_ns(rt) / 1e9
stop!(rt::RateTimer) = (rt.elapsed = time_ns(rt); rt.paused = true; rt)
start!(rt::RateTimer) = (rt.tlast = time_ns(); rt.paused = false; rt)
setrate!(rt::RateTimer, r) = (rt.rate = r; rt)
reset!(rt::RateTimer) = (rt.tlast = time_ns(); rt.elapsed = 0; rt) 
