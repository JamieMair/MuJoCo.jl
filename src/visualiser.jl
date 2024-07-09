# Creates definitions for the visualiser structs
module Visualiser

import ..LibMuJoCo
import ..LibMuJoCo: mjrContext, mjvOption, mjvCamera, mjvFigure, mjvScene, mjvPerturb
import ..Wrappers
import ..Wrappers: RendererContext, VisualiserOption, VisualiserCamera, VisualiserFigure, VisualiserScene, VisualiserPerturb
import ..Utils: alloc

function Wrappers.RendererContext()
    mj_context = alloc(mjrContext)
    LibMuJoCo.mjr_defaultContext(mj_context)
    return RendererContext(mj_context)
end
function Wrappers.VisualiserOption()
    mj_option = alloc(mjvOption)
    LibMuJoCo.mjv_defaultOption(mj_option)
    return VisualiserOption(mj_option)
end
function Wrappers.VisualiserCamera()
    mj_camera = alloc(mjvCamera)
    LibMuJoCo.mjv_defaultCamera(mj_camera)
    return VisualiserCamera(mj_camera)
end
function Wrappers.VisualiserFigure()
    mj_figure = alloc(mjvFigure)
    LibMuJoCo.mjv_defaultFigure(mj_figure)
    return VisualiserFigure(mj_figure)
end
function Wrappers.VisualiserScene()
    mj_scene = alloc(mjvScene)
    LibMuJoCo.mjv_defaultScene(mj_scene)
    return VisualiserScene(mj_scene)
end
function Wrappers.VisualiserPerturb()
    mj_perturb = alloc(mjvPerturb)
    LibMuJoCo.mjv_defaultPerturb(mj_perturb)
    return VisualiserPerturb(mj_perturb)
end

function test_visualiser end

"""
    visualise!(m::Model, d::Data; controller=nothing, trajectories=nothing, resolution=nothing)

Starts an interactive visualization of a MuJoCo model specified by an instance of `Model` and `Data`.

The visualizer has three "modes" that allow you to visualize passive dynamics, run a controller interactively, or play back recorded trajectories. The passive dynamics mode is always available, while the controller and trajectory modes are specified by the keyword arguments below.

Press `F1` for help after running the visualiser to print the available mouse/button options in the terminal. Switch between modes with `CTRL+RightArrow` and `CTRL+LeftArrow` (or `CMD` for Mac). The different visualiser modes are ordered as follows:
    
1. Controller mode (if `controller` keyword is provided)
2. Trajectory mode (if `trajectories` keyword is provided)
3. Passive mode (always available)

# Keywords

- `controller`: a callback function with the signature `controller(m, d)`, called at each timestep, that applies a control input to the system (or does any other operation you like).

- `trajectories`: a single trajectory or `Vector` of trajectories, where each trajectory is an `AbstractMatrix` of states with size `(nx, T)` where `nx = model.nq + model.nv + model.na` and `T` is the length of the trajectory. Note that each trajectory can have a different length.

- `resolution`: a specific initial window resolution, useful for recording videos. Set this to nothing to use default value of 2/3s of the screen size.

# Examples

```julia
using MuJoCo
install_visualiser() # Run this to install dependencies only once
init_visualiser()    # Load required dependencies into session

# Load a model
model, data = MuJoCo.sample_model_and_data()

# Simulate and record a trajectory
T = 200
nx = model.nq + model.nv + model.na
states = zeros(nx, T)
for t in 1:T
    states[:,t] = get_physics_state(model, data)
    step!(model, data)
end

# Define a controller
function ctrl!(m,d)
    d.ctrl .= 2*rand(m.nu) .- 1
end

# Run the visualiser
reset!(model, data)
visualise!(model, data, controller=ctrl!, trajectories = states)
```
"""
function visualise! end

export RendererContext, VisualiserOption, VisualiserCamera, VisualiserFigure, VisualiserScene, VisualiserPerturb, test_visualiser, visualise!

end


