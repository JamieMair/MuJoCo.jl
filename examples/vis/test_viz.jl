cd(@__DIR__)
using Pkg
Pkg.activate("..")

using MuJoCo

include("visualiser.jl")

# Load model, data, and viewer
model  = load_xml("../../models/humanoid.xml")
data   = init_data(model)
viewer = MuJoCoViewer(model, data)

# Loop and simulate for now
while !viewer.should_close

    # TODO: nsteps = Int(ceil((1.0/60.0) / m.opt.timestep))
    nsteps = Int(ceil((1.0/60.0) / 0.01))
    for _ in 1:nsteps
        step!(model, data)
    end

    render!(viewer, model, data)
end

LibMuJoCo.mj_deleteModel(model.internal_pointer)
LibMuJoCo.mj_deleteData(data.internal_pointer)
close_viewer!(viewer)

"""
Next steps: get mouse and button callbacks working

- Use register!(mngr::WindowManager, hs::EventHandler...) in glfw.jl
- Look at Lyceum code for how they set up the event handlers.
- Most of their code is based on the PhysicsState type
- Can modify it to take in Model and Data rather than their custom MJSim
- Need nice array handling first
"""