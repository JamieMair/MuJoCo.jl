# The following is from https://github.com/Lyceum/MuJoCo.jl/blob/master/src/MJCore/wrapper/cglobals.jl and https://github.com/Lyceum/MuJoCo.jl/blob/master/src/MJCore/util.jl#L132
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
    (:mjDISABLESTRING, (LibMuJoCo.mjNDISABLE, )),
    (:mjENABLESTRING, (LibMuJoCo.mjNENABLE, )),
    (:mjTIMERSTRING, (LibMuJoCo.mjNTIMER, )),
    (:mjLABELSTRING, (LibMuJoCo.mjNLABEL, )),
    (:mjFRAMESTRING, (LibMuJoCo.mjNFRAME, )),
    (:mjRNDSTRING, (3, LibMuJoCo.mjNRNDFLAG)),
    (:mjVISSTRING, (3, LibMuJoCo.mjNVISFLAG)),
)
    dims = map(Int, dims)
    T = SArray{Tuple{dims...}, Ptr{Cchar}, length(dims), prod(dims)}
    cref = CRef{name, T}()
    A = _unsafe_mj_string(cref[])
    @eval const $name = $A
end