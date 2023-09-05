```@meta
CurrentModule = MuJoCo
```

# MuJoCo

Documentation for [MuJoCo](https://github.com/JamieMair/MuJoCo.jl), a wrapper for DeepMind's [MuJoCo library](https://github.com/deepmind/mujoco).

Much of this project builds upon the work from Lyceum and their package [`Lyceum/MuJoCo.jl`](https://github.com/Lyceum/MuJoCo.jl), along with their visualisation project [`LyceumMuJoCoViz`](https://github.com/Lyceum/LyceumMuJoCoViz.jl).

This wrapper aims to provide a simple, performant interface to the MuJoCo C library, allowing direct manipulation of the data used by the C engine for optimal performance. See the [API](@ref) page for our wrapped functions.

We also provide a thin wrapper directly around the MuJoCo C library, which directly wraps all functions exported by the `libmujoco.h` header file. You can view the structs and functions available in the [LibMuJoCo](@ref) page.
