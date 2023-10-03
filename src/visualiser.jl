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
    visualise!(m::Model, d::Data; controller=nothing)

Starts an interactive visualization of a MuJoCo model specified by an instance of `Model` and `Data`.

The visualizer has two "modes" that allow you to visualize passive dynamics or run a controller interactively. The passive dynamics mode is always available, while the controller mode is specified by the keyword argument below.

Press F1 for help after running the visualiser to print the available options in a terminal.

# Keywords

- `controller`: a callback function with the signature `controller(m, d)`, called at each timestep, that applies a control input to the system (or does any other operation you like).

# Examples

```julia
using MuJoCo
install_visualiser() # Run this to install dependencies only once
init_visualiser()    # Load required dependencies into session

# Load a model
model, data = MuJoCo.sample_model_and_data()

# Define a controller
function ctrl!(m,d)
    d.ctrl .= 2*rand(m.nu) .- 1
end

# Run the visualiser
visualise!(model, data, controller=ctrl!)
```
"""
function visualise! end

export RendererContext, VisualiserOption, VisualiserCamera, VisualiserFigure, VisualiserScene, VisualiserPerturb, test_visualiser, visualise!

end


