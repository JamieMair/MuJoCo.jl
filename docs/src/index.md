```@meta
CurrentModule = MuJoCo
```

# MuJoCo.jl Documentation

*Documentation for [MuJoCo.jl](https://github.com/JamieMair/MuJoCo.jl), a wrapper for DeepMind's [MuJoCo library](https://github.com/deepmind/mujoco).*

Welcome to the documentation for `MuJoCo.jl`. This package contains a Julia wrapper for  DeepMind's general-purpose physics engine, allowing users to simulate robotic systems, their controllers, and their interactions with surrounding environments, all in native Julia.

This wrapper aims to provide a simple, performant interface to the [MuJoCo C library](https://github.com/deepmind/mujoco), allowing direct manipulation of the data used by the C engine for optimal performance. See the [MuJoCo API](@ref) page for our wrapped functions. We also provide a thin wrapper directly around the MuJoCo C library, which directly wraps all functions exported by the `libmujoco.h` header file. You can view the structs and functions available in the [LibMuJoCo Index](@ref) page.

## Version

The version of MuJoCo this package uses is always made to match the version of `MuJoCo_jll`, checked via
```@example
import Pkg
Pkg.status("MuJoCo_jll"; mode=Pkg.PKGMODE_MANIFEST)
```

## Introduction

- [Getting Started](@ref)
- [Package Overview](@ref)

## Examples

- [Balancing a Cart-Pole](@ref)
- [Humanoid LQR](@ref)

## Contributing

- [Contributing to the Package](@ref)

## Library

- [MuJoCo API](@ref)
- [LibMuJoCo Index](@ref)
- [LibMuJoCo API](@ref)

## Acknowledgements

Much of this project builds upon the work from [Lyceum](https://github.com/Lyceum) and their package [`Lyceum/MuJoCo.jl`](https://github.com/Lyceum/MuJoCo.jl), along with their visualisation project [`LyceumMuJoCoViz`](https://github.com/Lyceum/LyceumMuJoCoViz.jl). We would like to thank the authors of these packages for their amazing work in originally bringing MuJoCo to Julia.