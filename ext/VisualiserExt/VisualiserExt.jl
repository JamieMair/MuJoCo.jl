# Many elements of this file are taken from https://github.com/Lyceum/LyceumMuJoCoViz.jl

module VisualiserExt

if isdefined(Base, :get_extension)
    using MuJoCo
    import MuJoCo.LibMuJoCo
    import MuJoCo.Visualiser
    import GLFW
    using FFMPEG
    using PrettyTables: pretty_table
    using Printf: @printf
    using StaticArrays
else
    using ..MuJoCo
    import ..MuJoCo.LibMuJoCo
    import ..MuJoCo.Visualiser
    import ..GLFW
    using ..FFMPEG
    using ..PrettyTables: pretty_table
    using ..Printf: @printf
    using ..StaticArrays
end

include("visualiser.jl")

function __init__()
    if Threads.nthreads() == 1
        @warn "The visualiser for MuJoCo.jl is designed to run multi-threaded, but the current Julia session was started with only one thread. Degraded performance will occur. To enable multi-threading, set JULIA_NUM_THREADS to a value greater than 1 before starting Julia."
    end
    return nothing
end

function MuJoCo.Visualiser.test_visualiser()
    root_path = normpath(joinpath(@__DIR__, "..", ".."))

    # Load model, data
    model = load_xml(joinpath(root_path, "models", "cartpole.xml"))
    data  = init_data(model)

    # Change initial conditions
    data.qpos .= @SVector [0, 0.0001]
    data.qvel .= @SVector [0.01, 0]

    # Random control action
    function ctrl!(m::Model, d::Data) 
        d.ctrl .= 2*rand(m.nu) .- 1
        return nothing
    end

    # Visualise the model with this controller
    visualise(model, data, controller=ctrl!)
end

end