# MuJoCo

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://JamieMair.github.io/MuJoCo.jl/stable/)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://JamieMair.github.io/MuJoCo.jl/dev/)
[![Build Status](https://github.com/JamieMair/MuJoCo.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JamieMair/MuJoCo.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JamieMair/MuJoCo.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JamieMair/MuJoCo.jl)

`MuJoCo.jl` aims to provide a native Julia interface to the popular [`mujoco`](https://github.com/google-deepmind/mujoco) library. When installed, `MuJoCo.jl` will automatically download the appropriate build of the C `mujoco` library and provide a wrapper to interface with this library. We have provided some native Julia wrappers for convenient use of the package, while also allowing direct use of the C library through a thin wrapper.

As `MuJoCo.jl` is currently under development. While the core functionality is likely usable at this time, the API is likely to change rapidly. There will likely be some bugs and issues that are currently being worked on. All of our development is done in public, and you can check out the current status by going through the [Issues](https://github.com/JamieMair/MuJoCo.jl/issues) page.

## Getting Started

To install `MuJoCo.jl`, simply run the following code in a Julia REPL:
```julia
import Pkg; Pkg.add(url="https://github.com/JamieMair/MuJoCo.jl")
```
This will download and install the package, along with the underlying C library. We highly recommend activating a project to manage dependencies - see the [docs](https://docs.julialang.org/en/v1/stdlib/Pkg/) for more information.

Once installed, you can load the package in the REPL by typing
```julia
using MuJoCo
```
From here, you can load a test model and data set using
```julia
model, data = MuJoCo.sample_model_and_data();
```
You can directly access data from these structs, e.g. 
```julia
model.qpos0 # Returns a 28x1 array 
data.xpos # Returns a 17x3 array
```
You can directly read and write from these arrays. We cannot overwrite an array directly, but we can use Julia's broadcasting to set values:
```julia
data.xpos .= 0
```
To run the simulation forwards, you can use the `step!` function
```julia
step!(model, data)
```
The `!` indicates an in-place operation that mutates the inputs (in this case the `data` struct).

## Named Access

Many MuJoCo models will attach names to the various joints and bodies that make up the model. It is very convenient to be able to select a single part of the body and access information that may be contained in the larger arrays of `model` or `data`. `MuJoCo.jl` provides a few wrapper functions that allow us to access the underlying data.

Take the example of the torso object
```julia
import MuJoCo as MJ
model, data = MJ.sample_model_and_data();
MJ.step!(model, data);
torso = MJ.body(data, "torso")
```
If you display the `torso` object in the REPL, you can see what fields can be accessed using this object, i.e.
```text
DataBody:
id:       1
name:     "torso"
applied:  (6 x  )       Float64 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (4 s.f.)
xpos:     (3 x  )       Float64 [0.0, 0.0, 1.282] (4 s.f.)
xquat:    (4 x  )       Float64 [1.0, 0.0, 0.0, 0.0] (4 s.f.)
xmat:     (9 x  )       Float64 [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] (4 s.f.)
xipos:    (3 x  )       Float64 [-0.003864, 0.0, 1.236] (4 s.f.)
ximat:    (9 x  )       Float64 [0.9965, 0.0, 0.08305, 0.0, 1.0, 0.0, -0.08305, 0.0, 0.9965] (4 s.f.)
com:      (3 x  )       Float64 [0.01569, 0.0, 0.8498] (4 s.f.)
cinert:   (10 x  )      Float64 [0.9224, 0.9055, 0.03353, 0.0, 0.04248, 0.0, -0.1144, 0.0, 2.258, 5.854] (4 s.f.)
crb:      (10 x  )      Float64 [7.797, 7.482, 0.905, 0.0, -0.4014, 1.356e-18, 1.874e-16, 0.0, 8.438e-15, 40.84] (4 s.f.)
cvel:     (6 x  )       Float64 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (4 s.f.)
linvel:   (3 x  )       Float64 [0.0, 0.0, 0.0] (4 s.f.)
angmom:   (3 x  )       Float64 [0.0, 0.0, 0.0] (4 s.f.)
cacc:     (6 x  )       Float64 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (4 s.f.)
int:      (6 x  )       Float64 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (4 s.f.)
ext:      (6 x  )       Float64 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (4 s.f.)
```
So `torso.com` will give you a $3$-dimensional vector representing the centre of mass of the torso. There are several other functions that will generate a named wrapper around each object.

## Arrays

It is highly important to keep in mind that the underlying C API expects arrays to be row-major. However, all Julia arrays are column-major by default. When passing arrays to the wrapped C functions, be careful about this distinction. We have plans to provide a nicer API to handle this use case.

All arrays returned by our API are usually of the type `transpose(::UnsafeArray)`, which is how we represent the row-major arrays. This allows you to safely use these arrays alongside any column-major arrays in your Julia code. Keeping this type will also allow for performance specialisation on BLAS operations. 

## Tips

When writing code, keep a REPL open with a sample `data` and `model` object (such as generated in the above examples), and use tab-completition (double tap `tab`) to see what information is available. Check the online docs to read the documentation for the parameters.

Also, all functions or structs that begin with `mj` are directly using the underlying C API.

## Acknowledgements

This work is based on an older [repository](https://github.com/Lyceum/MuJoCo.jl) from Lyceum. Many thanks to the authors of this original package that served as inspiration for this version.