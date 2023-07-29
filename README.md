# MuJoCo

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://JamieMair.github.io/MuJoCo.jl/stable/)
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://JamieMair.github.io/MuJoCo.jl/dev/)
[![Build Status](https://github.com/JamieMair/MuJoCo.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JamieMair/MuJoCo.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JamieMair/MuJoCo.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JamieMair/MuJoCo.jl)


This work is based on an older [repository](https://github.com/Lyceum/MuJoCo.jl) from Lyceum. Many thanks to the authors for providing this repository as a base for our work.

## Development

You need to add the newer JLL to use this library.
```julia
pkg] rm MuJoCo_jll
pkg] add https://github.com/JamieMair/MuJoCo_jll
```
You will need to do this in the `gen` folder's project as well. To re-generate the bindings, `cd` into the `gen` folder and run 
```julia
julia --project generator.jl
```

### Running binaries

To run the binaries, load the JLL and run:
```julia
using MuJoCo_jll

mujoco_simulate() do bin
    run(`$bin /path/to/model.xml`)
end
```