module MuJoCo
using UnsafeArrays

include("LibMuJoCo/LibMuJoCo.jl")
include("utils.jl")

using .LibMuJoCo
module Wrappers
    using ..LibMuJoCo
    include("wrappers.jl")
    include("visualiser_wrappers.jl")
    include("named_access.jl")
end

include("visualiser.jl")

using .Wrappers


import .Visualiser: visualise!


export init_data, step!, forward!, timestep
export init_visualiser, install_visualiser, visualise!

include("io.jl")

"""
    init_data(model::Model)

Creates an instance of the data required for the [`step!`](@ref) simulation.

Returns a `Data` object, wrapping the underlying `mjData` object.
"""
function init_data(model::Model)
    data_ptr = mj_makeData(model)
    return Data(data_ptr, model) # Requires a reference to the model to get array sizes
end
"""
    step!(model::Model, data::Data)

Runs the simulation forward one time step, modifying the underlying `data` object.
"""
function step!(model::Model, data::Data)
    mj_step(model, data)
end
function forward!(model::Model, data::Data)
    mj_forward(model, data)
end
timestep(model::Model) = model.opt.timestep # Useful

# Handle backwards compatibility
if !isdefined(Base, :get_extension)
    using Requires
end

"""
    init_visualiser()

Loads the packages necessary for the running the visualiser.

Add the following packages to your project to be able to use
the visualisation features, or run "install_visualiser()".

# Packages:
- FFMPEG
- GLFW
- Observables
- PrettyTables
- Printf
- StaticArrays
"""
function init_visualiser()
    @eval Main using FFMPEG, GLFW, Observables, PrettyTables, Printf, StaticArrays
    if isdefined(Base, :get_extension)
        @eval Main Base.retry_load_extensions()
    end
end
"""
    install_visualiser()

Installs the necessary packages for the running the visualiser
into the current running environment.

# Packages:
- FFMPEG
- GLFW
- Observables
- PrettyTables
- Printf
- StaticArrays
"""
function install_visualiser()
    @eval Main import Pkg
    @eval Main Pkg.add(["FFMPEG", "GLFW", "Observables", "PrettyTables", "Printf", "StaticArrays"])
end

@static if !isdefined(Base, :get_extension)
function __init__()
    @static if !isdefined(Base, :get_extension)
        @require FFMPEG = "c87230d0-a227-11e9-1b43-d7ebe4e7570a" begin
            @require GLFW = "f7f18e0c-5ee9-5ccd-a5bf-e8befd85ed98" begin
                @require Observables = "510215fc-4207-5dde-b226-833fc4488ee2" begin
                    @require PrettyTables = "08abe8d2-0d0c-5749-adfa-8a2ac140af0d" begin
                        @require Printf = "de0858da-6303-5e67-8744-51eddeeeb8d7" begin
                            @require StaticArrays = "90137ffa-7385-5640-81b9-e52037218182" begin
                                include("../ext/VisualiserExt/VisualiserExt.jl")
                            end
                        end
                    end
                end
            end
        end
    end
end
end

end