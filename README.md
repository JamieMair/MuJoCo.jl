# MuJoCo

[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://JamieMair.github.io/MuJoCo.jl/dev/)
[![Build Status](https://github.com/JamieMair/MuJoCo.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JamieMair/MuJoCo.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JamieMair/MuJoCo.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JamieMair/MuJoCo.jl)

Welcome to `MuJoCo.jl`! This package contains a Julia wrapper for  DeepMind's general-purpose physics engine [MuJoCo](https://github.com/google-deepmind/mujoco), allowing users to simulate robotic systems, their controllers, and their interactions with surrounding environments, all in native Julia. Please visit our [docs page](https://jamiemair.github.io/MuJoCo.jl/dev/) for more information.

## Installation

To install `MuJoCo.jl`, run the following code in a Julia REPL:
```julia
import Pkg
Pkg.add("MuJoCo.jl")
```
This will download and install the package, along with the underlying C library. You should now be able to simulate MuJoCo models and visualise their motion. The following example loads a humanoid robot model and simulates its motion under passive or controlled dynamics through our in-built visualiser.
```julia
using MuJoCo
install_visualiser() # Run this to install dependencies only once
init_visualiser()    # Load required dependencies into session

# Load a model of a humanoid
model, data = MuJoCo.sample_model_and_data()

# Define a controller
function ctrl!(model, data)
    data.ctrl .= 2*rand(model.nu) .- 1
end

# Run the visualiser (press F1 for available options)
visualise!(model, data, controller=ctrl!)
```
A detailed walkthrough of this example can be found on our [Getting Started](https://jamiemair.github.io/MuJoCo.jl/dev/intro/getting_started/) page.

## Acknowledgements

Much of this project builds upon the work from [Lyceum](https://github.com/Lyceum) and their package [`Lyceum/MuJoCo.jl`](https://github.com/Lyceum/MuJoCo.jl), along with their visualisation project [`LyceumMuJoCoViz`](https://github.com/Lyceum/LyceumMuJoCoViz.jl). We would like to thank the authors of these packages for their amazing work in originally bringing MuJoCo to Julia.
