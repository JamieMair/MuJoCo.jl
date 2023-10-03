# Getting Started

## Installation

To install `MuJoCo.jl`, simply run the following code in a Julia REPL:
```julia
import Pkg
Pkg.add(url="https://github.com/JamieMair/MuJoCo.jl")
```
This will download and install the package, along with the underlying C library. We highly recommend activating a project to manage dependencies - see the [docs](https://docs.julialang.org/en/v1/stdlib/Pkg/) for more information.

Once installed, you can load the package in the REPL just like any other Julia package. To use the visualiser and see your MuJoCo models in action, you will need to install some additional dependencies with
```julia
using MuJoCo
install_visualiser()
```

## Basic Usage

You should now be able to load in and play around with any MuJoCo model of your choosing. You can create your own MuJoCo models with the [MJCF modelling language](https://mujoco.readthedocs.io/en/stable/XMLreference.html), but for now we'll load in a model of a humanoid for demonstration purposes.
```@example demo
using MuJoCo

model, data = MuJoCo.sample_model_and_data()
println(typeof(model)," ", typeof(data))
```
The `Model` and `Data` types encode all the information required to simulate a model in MuJoCo, and are wrappers of the `mjModel` and `mjData` structs in the C API, respectively. We can directly access any data from these structs:
```@example demo
println(model.opt.timestep) # Simulation timestep
println(data.qpos)          # Position of all joints
```
We can also directly read and write from/to these fields. However, we cannot directly overwrite any arrays. Instead, we can use Julia's broadcasting to set values as we see fit. Let's write a function that inputs random control torques to the humanoid's joints.
```@example demo
function random_controller!(m::Model, d::Data)
    nu = m.nu
    d.ctrl .= 2*rand(nu) .- 1
    return nothing
end
```
We can now simulate motion of the humanoid under this control scheme. Let's simulate over 100 time-steps.
```@example demo
for t in 1:100
    random_controller!(model, data)
    step!(model, data)
end
```
At each time-step, `random_controller!` sets the control signal to some random value, and `step!` calls the MuJoCo physics engine to simulate the response of the system. `step!` directly modifies the `data` struct. For example, looking at `data.qpos` again shows that the joints have all moved.
```@example demo
println(data.qpos)
```
After finishing our initial simulations, we can re-set the model back to its starting position by calling `mj_resetData`, one of the underlying C library functions. Any of the functions listed in the [LibMuJoCo Index](@ref) can be used just as they are described in the [MuJoCo documentation](https://mujoco.readthedocs.io/en/stable/APIreference/index.html).
```@example demo
mj_resetData(model, data)
println(data.qpos)
```

## Visulaising a Model

Of course, the best way to understand what our random controller is doing is to visualise the humanoid model. To use the visualiser, simply initialise it and call [`visualise!`](@ref) with the `controller` keyword set to our `random_controller!` function.
```julia
init_visualiser()
visualise!(model, data, controller=random_controller!)
```
![](humanoid_random_demo.mp4)
Press F1 for help after running the visualiser to print the available options in a terminal. Some of the most interesting are:
- Press `CTRL+RightArrow` (or `CMD` for Mac) to turn on the controller
- Press `SPACE` to pause/un-pause
- Double-click on an object select it
- `CTRL+RightClick` and drag to apply a force
- Press `ESC` to exit the simulation

Happy visualising!