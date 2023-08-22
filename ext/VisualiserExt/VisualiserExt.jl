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

function MuJoCo.Visualiser.test_visualiser()
    root_path = normpath(joinpath(@__DIR__, "..", ".."))

    # Load model, data
    model  = load_xml(joinpath(root_path, "models", "cartpole.xml"))
    data   = init_data(model)

    # Change initial conditions
    data.qpos .= @SVector [0, 0.0001]
    data.qvel .= @SVector [0.01, 0]

    # Random control action
    function ctrl!(m::Model, d::Data) 
        d.ctrl .= 2*rand(m.nu) .- 1
        return nothing
    end

    # Load the viewer
    viewer = MuJoCoViewer(model, data, controller=ctrl!)

    # Loop and simulate for now
    fps = 60
    frametime = 1 / fps
    while !viewer.should_close

        # TODO: Throttle visualisation inside render!() somewhere
        previous_time = data.time
        while (data.time - previous_time < frametime)
            ctrl!(model, data)
            step!(model, data)
        end

        render!(viewer, model, data)
    end
    close_viewer!(viewer)
end

end