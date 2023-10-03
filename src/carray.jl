
"""
    mj_array([eltype=mjtNum], dims...)

Allocates an array compatible with the underlying MuJoCo C API.

The C API treats arrays as row-major, while by default arrays in
Julia are column-major. This function will create an array which
is accessed in a row-major way, but can be treated by a normal 
array in your Julia code.

# Arguments
- **eltype**: The element type of the array. Defaults to mjtNum (typically Float64).
- **dims**: The dimensionality of output array. Can either be a tuple of integers or a series of integers.
"""
function mj_array(eltype, dims...)
    return transpose(Array{eltype}(undef, reverse(dims)...))
end
function mj_array(dims...)
    eltype=LibMuJoCo.mjtNum
    mj_array(eltype, dims...)
end
mj_array(eltype, dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_array(eltype, dims...)
mj_array(dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_array(eltype, dims...)


"""
    mj_zeros([eltype=mjtNum], dims...)

Allocates an array full of zeros, compatible with the underlying
MuJoCo C API.

The C API treats arrays as row-major, while by default arrays in
Julia are column-major. This function will create an array which
is accessed in a row-major way, but can be treated by a normal 
array in your Julia code.

# Arguments
- **eltype**: The element type of the array. Defaults to mjtNum (typically Float64).
- **dims**: The dimensionality of output array. Can either be a tuple of integers or a series of integers.
"""
function mj_zeros(eltype, dims...)
    array = transpose(Array{eltype}(undef, reverse(dims)...))
    fill!(array, zero(eltype(array)))
    return array
end
function mj_zeros(dims...)
    eltype=LibMuJoCo.mjtNum
    mj_zeros(eltype, dims...)
end
mj_zeros(eltype, dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_zeros(eltype, dims...)
mj_zeros(dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_zeros(eltype, dims...)