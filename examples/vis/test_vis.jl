using MuJoCo
include("visualiser.jl")

function main()
    root_path = normpath(joinpath(@__DIR__, "..", ".."))

    # Load model, data, and viewer
    model  = load_xml(joinpath(root_path, "models", "cartpole.xml"))
    data   = init_data(model)
    viewer = MuJoCoViewer(model, data)

    # Change initial conditions
    data.qpos .= [0, 0.0001]
    data.qvel .= [0.01, 0]

    # Loop and simulate for now
    fps = 60
    frametime = 1 / fps
    while !viewer.should_close

        # TODO: Throttle visualisation inside render!() somewhere
        previous_time = data.time
        while (data.time - previous_time < frametime)
            step!(model, data)
        end

        render!(viewer, model, data)
    end
    close_viewer!(viewer)

    LibMuJoCo.mj_deleteModel(model.internal_pointer)
    LibMuJoCo.mj_deleteData(data.internal_pointer)
end

main()

"""
Next steps: get mouse and button callbacks working

- Use register!(mngr::WindowManager, hs::EventHandler...) in glfw.jl
- Look at Lyceum code for how they set up the event handlers.
- Most of their code is based on the PhysicsState type
- Can modify it to take in Model and Data rather than their custom MJSim
- Need nicer pointer handling?
"""