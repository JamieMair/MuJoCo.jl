
"""
    mj_array([element_type=mjtNum], dims...)

Allocates an array compatible with the underlying MuJoCo C API.

The C API treats arrays as row-major, while by default arrays in
Julia are column-major. This function will create an array which
is accessed in a row-major way, but can be treated by a normal 
array in your Julia code.

# Arguments
- **element_type**: Defaults to mjtNum (typically Float64).
- **dims**: The dimensionality of output array. Can either be a tuple of integers or a series of integers.
"""
function mj_array(element_type::DataType, dims...)
    return transpose(Array{element_type}(undef, reverse(dims)...))
end
function mj_array(dims...)
    element_type=LibMuJoCo.mjtNum
    mj_array(element_type, dims...)
end
mj_array(element_type::DataType, dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_array(element_type, dims...)
mj_array(dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_array(element_type, dims...)


"""
    mj_zeros([element_type=mjtNum], dims...)

Allocates an array full of zeros, compatible with the underlying
MuJoCo C API.

The C API treats arrays as row-major, while by default arrays in
Julia are column-major. This function will create an array which
is accessed in a row-major way, but can be treated by a normal 
array in your Julia code.

# Arguments
- **element_type**: The element type of the array. Defaults to mjtNum (typically Float64).
- **dims**: The dimensionality of output array. Can either be a tuple of integers or a series of integers.
"""
function mj_zeros(element_type::DataType, dims...)
    array = Array{element_type}(undef, reverse(dims)...)
    fill!(array, zero(element_type))
    return transpose(array)
end
function mj_zeros(dims...)
    element_type=LibMuJoCo.mjtNum
    mj_zeros(element_type, dims...)
end
mj_zeros(element_type::DataType, dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_zeros(element_type, dims...)
mj_zeros(dims::T) where {N, T<:NTuple{N,<:Integer}} = mj_zeros(element_type, dims...)