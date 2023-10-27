module VisualiserExt
# Many elements of this file are taken from https://github.com/Lyceum/LyceumMuJoCoViz.jl

############ Module dependencies ############

import Base: @lock, @lock_nofail

if isdefined(Base, :get_extension)
    using MuJoCo
    using MuJoCo_jll
    using MuJoCo.Wrappers
    using MuJoCo.LibMuJoCo
    using MuJoCo.LibMuJoCo: mjrRect
    using MuJoCo.Visualiser

    using BangBang
    using FFMPEG
    using GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
    using Observables: Observable, on, off
    using PrettyTables: pretty_table
    using Printf: @printf
    using StaticArrays
    using UnsafeArrays: UnsafeArray
else
    using ..MuJoCo
    using ..MuJoCo_jll
    using ..MuJoCo.Wrappers
    using ..MuJoCo.LibMuJoCo
    using ..MuJoCo.LibMuJoCo: mjrRect
    using ..MuJoCo.Visualiser

    using ..BangBang
    using ..FFMPEG
    using ..GLFW: GLFW, Window, Key, Action, MouseButton, GetKey, RELEASE, PRESS, REPEAT
    using ..Observables: Observable, on, off
    using ..PrettyTables: pretty_table
    using ..Printf: @printf
    using ..StaticArrays
    using ..UnsafeArrays: UnsafeArray
end


############ Constants ############

const Maybe{T} = Union{T, Nothing}  # From LyceumBase.jl
const MAXGEOM = 10000               # preallocated geom array in mjvScene
const MIN_REFRESHRATE = 30          # minimum rate when sim can't run at native refresh rate
const RNDGAMMA = 0.9

const RES_HD = (1280, 720)
const RES_FHD = (1920, 1080)
const RES_XGA = (1024, 768)
const RES_SXGA = (1280, 1024)


############ C Globals ############
include("gclobals.jl")

############ Includes ############

include("util.jl")
include("glfw.jl")
include("ratetimer.jl")
include("types.jl")
include("functions.jl")
include("modes.jl")
include("defaulthandlers.jl")

include("visualiser.jl")


############ Functions ############

function __init__()
    if Threads.nthreads() == 1
        @warn "The visualiser for MuJoCo.jl is designed to run multi-threaded, but the current Julia session was started with only one thread. Degraded performance will occur. To enable multi-threading, set JULIA_NUM_THREADS to a value greater than 1 before starting Julia."
    end
    return nothing
end

function MuJoCo.Visualiser.test_visualiser()
    # Load model, data
    model = load_model(MuJoCo.humanoid_model_file())
    data  = init_data(model)

    # Random control action
    function ctrl!(m::Model, d::Data) 
        d.ctrl .= 2*rand(m.nu) .- 1
        return nothing
    end

    # Visualise the model with this controller
    visualise!(model, data, controller=ctrl!)
end

end # end VisualiserExt