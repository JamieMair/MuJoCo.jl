cd(@__DIR__)
using Pkg
Pkg.activate("..")

using MuJoCo

include("visualiser.jl")


function main()
    # Load model, data, and viewer
    model  = load_xml("../../models/cartpole.xml")
    data   = init_data(model)
    viewer = MuJoCoViewer(model, data)

    # Change initial conditions
    d.qpos .= [0, 0.1]

    # Loop and simulate for now
    while !viewer.should_close

        # TODO: Throttle visualisation inside render!() somewhere
        nsteps = Int(ceil((1.0/60.0) / m.opt.timestep))
        for _ in 1:nsteps
            step!(model, data)
        end

        render!(viewer, model, data)
    end

    LibMuJoCo.mj_deleteModel(model.internal_pointer)
    LibMuJoCo.mj_deleteData(data.internal_pointer)
    close_viewer!(viewer)
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