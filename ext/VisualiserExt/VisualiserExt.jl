# Many elements of this file are taken from https://github.com/Lyceum/LyceumMuJoCoViz.jl

module VisualiserExt


############ Module dependencies ############

import Base: @lock, @lock_nofail

if isdefined(Base, :get_extension)
    using MuJoCo
    using MuJoCo_jll
    import MuJoCo.LibMuJoCo
    import MuJoCo.LibMuJoCo: Model, Data, mjrRect, mjr_render
    using MuJoCo.Visualiser

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
    import ..MuJoCo.LibMuJoCo
    import ..MuJoCo.LibMuJoCo: Model, Data, mjrRect, mjr_render
    using ..MuJoCo.Visualiser

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

# The following is from https://github.com/Lyceum/MuJoCo.jl/blob/master/src/MJCore/wrapper/cglobals.jl and https://github.com/Lyceum/MuJoCo.jl/blob/master/src/MJCore/util.jl#L132

# TODO: Move this to the codegen or MuJoCo.jl? @Jamie where do you reckon is best?

struct CRef{name, T}
    CRef{name, T}() where {name, T} = new{name::Symbol, T}()
end

Base.getindex(::CRef{name,T}) where {name,T} = loadglobal(Val(name), T)

function loadglobal(::Val{name}, ::Type{T}, i::Integer = 1) where {name,T}
    Base.unsafe_load(getglobal(Val(name), T))
end

getglobal(::Val{name}, ::Type{T}) where {name,T} = cglobal((name, libmujoco), T)

function _unsafe_mj_string(A::AbstractArray{Ptr{Cchar}})
    map(A) do x
        replace(Base.unsafe_string(x), "&" => "")
    end
end

for (name, dims) in (
    # (:mjDISABLESTRING, (mjNDISABLE, )),
    # (:mjENABLESTRING, (mjNENABLE, )),
    # (:mjTIMERSTRING, (mjNTIMER, )),
    (:mjLABELSTRING, (LibMuJoCo.mjNLABEL, )),
    (:mjFRAMESTRING, (LibMuJoCo.mjNFRAME, )),
    (:mjRNDSTRING, (3, LibMuJoCo.mjNRNDFLAG)),
    (:mjVISSTRING, (3, LibMuJoCo.mjNVISFLAG)),
)
    # TODO: is this safe to do at module initialization time?
    dims = map(Int, dims)
    T = SArray{Tuple{dims...}, Ptr{Cchar}, length(dims), prod(dims)}
    cref = CRef{name, T}()
    A = _unsafe_mj_string(cref[])
    @eval const $name = $A
end


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
    root_path = normpath(joinpath(@__DIR__, "..", ".."))

    # Load model, data
    model = load_xml(joinpath(root_path, "models", "humanoid.xml"))
    data  = init_data(model)

    # # Change initial conditions for cartpole
    # data.qpos .= @SVector [0, 0.0001]
    # data.qvel .= @SVector [0.01, 0]

    # Random control action
    function ctrl!(m::Model, d::Data) 
        d.ctrl .= 2*rand(m.nu) .- 1
        return nothing
    end

    # Visualise the model with this controller
    visualise!(model, data, controller=ctrl!)
end

end # end VisualiserExt