import LinearAlgebra
import .LibMuJoCo
function column_major_warning_string(variable_name)
    return "$variable_name is stored in column-major order (Julia default), but mujoco expects arrays in row-major order. Use helper functions to generate row-major arrays and see documentation for more details."
end

export mju_printMat,
    mj_solveM,
    mj_solveM2,
    mj_rne,
    mj_constraintUpdate,
    mj_getState,
    mj_setState,
    mj_mulJacVec,
    mj_mulJacTVec,
    mj_jac,
    mj_jacBody,
    mj_jacBodyCom,
    mj_jacSubtreeCom,
    mj_jacGeom,
    mj_jacSite,
    mj_jacPointAxis,
    mj_fullM,
    mj_mulM,
    mj_mulM2,
    mj_addM,
    mj_applyFT,
    mj_differentiatePos,
    mj_integratePos,
    mj_normalizeQuat,
    mj_ray,
    mju_zero,
    mju_fill,
    mju_copy,
    mju_sum,
    mju_L1,
    mju_scl,
    mju_add,
    mju_sub,
    mju_addTo,
    mju_subFrom,
    mju_addToScl,
    mju_addScl,
    mju_normalize,
    mju_norm,
    mju_dot,
    mju_mulMatVec,
    mju_mulMatTVec,
    mju_mulVecMatVec,
    mju_transpose,
    mju_symmetrize,
    mju_eye,
    mju_mulMatMat,
    mju_mulMatMatT,
    mju_mulMatTMat,
    mju_sqrMatTD,
    mju_cholFactor,
    mju_cholSolve,
    mju_cholUpdate,
    mju_cholFactorBand,
    mju_cholSolveBand,
    mju_band2Dense,
    mju_dense2Band,
    mju_bandMulMatVec,
    mju_boxQP,
    mju_encodePyramid,
    mju_decodePyramid,
    mju_isZero,
    mju_f2n,
    mju_n2f,
    mju_d2n,
    mju_n2d,
    mju_insertionSort,
    mju_insertionSortInt,
    mjd_transitionFD,
    mjd_inverseFD


"""
    mju_printMat(mat)

Print matrix to screen.

# Arguments
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
"""
function mju_printMat(mat::AbstractArray{Float64,2})
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    return LibMuJoCo.mju_printMat(mat, size(mat, 1), size(mat, 2))

end
"""
    mj_solveM(m, d, x, y)

Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y

# Arguments
- m::Model -> Constant.
- d::Data
- x::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- y::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
"""
function mj_solveM(m, d, x::AbstractArray{Float64,2}, y::AbstractArray{Float64,2})
    if !(typeof(x) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("x")
    end
    if !(typeof(y) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("y")
    end

    if (size(x, 1) != size(y, 1))
        throw(ArgumentError("the first dimension of x and y should be of the same size"))
    end
    if (size(x, 2) != m.nv)
        throw(ArgumentError("the last dimension of x should be of size nv"))
    end
    if (size(y, 2) != m.nv)
        throw(ArgumentError("the last dimension of y should be of size nv"))
    end
    return LibMuJoCo.mj_solveM(m, d, x, y, size(y, 1))

end
"""
    mj_solveM2(m, d, x, y)

Half of linear solve:  x = sqrt(inv(D))*inv(L')*y

# Arguments
- m::Model -> Constant.
- d::Data
- x::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- y::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
"""
function mj_solveM2(m, d, x::AbstractArray{Float64,2}, y::AbstractArray{Float64,2})
    if !(typeof(x) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("x")
    end
    if !(typeof(y) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("y")
    end

    if (size(x, 1) != size(y, 1))
        throw(ArgumentError("the first dimension of x and y should be of the same size"))
    end
    if (size(x, 2) != m.nv)
        throw(ArgumentError("the last dimension of x should be of size nv"))
    end
    if (size(y, 2) != m.nv)
        throw(ArgumentError("the last dimension of y should be of size nv"))
    end
    return LibMuJoCo.mj_solveM2(m, d, x, y, size(y, 1))

end
"""
    mj_rne(m, d, flg_acc, result)

RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.

# Arguments
- m::Model -> Constant.
- d::Data
- flg_acc::Int32
- result::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- result should be a vector, not a matrix.
- result should have length nv

"""
function mj_rne(
    m,
    d,
    flg_acc::Integer,
    result::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(result) <: AbstractArray{Float64,2} && count(==(1), size(result)) < 1
        throw(ArgumentError("result should be a vector, not a matrix."))
    end

    if (length(result) != m.nv)
        throw(ArgumentError("result should have length nv"))
    end
    return LibMuJoCo.mj_rne(m, d, flg_acc, result)

end
"""
    mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)

Compute efc*state, efc*force, qfrc_constraint, and (optionally) cone Hessians. If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.

# Arguments
- m::Model -> Constant.
- d::Data
- jar::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- cost::Vector{Float64} -> An optional vector of size 1.
- flg_coneHessian::Int32

# Constraints
- jar should be a vector, not a matrix.
- cost should be a vector of size 1
- cost should be a vector of size 1.
- size of jar should equal nefc

"""
function mj_constraintUpdate(
    m,
    d,
    jar::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    cost::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    flg_coneHessian::Integer,
)
    if typeof(jar) <: AbstractArray{Float64,2} && count(==(1), size(jar)) < 1
        throw(ArgumentError("jar should be a vector, not a matrix."))
    end
    if !isnothing(cost) && length(cost) != 1
        throw(ArgumentError("cost should be a vector of size 1"))
    end
    if !isnothing(cost) &&
       typeof(cost) <: AbstractArray{Float64,2} &&
       count(==(1), size(cost)) < 1
        throw(ArgumentError("cost should be a vector of size 1."))
    end

    if (length(jar) != d.nefc)
        throw(ArgumentError("size of jar should equal nefc"))
    end
    return LibMuJoCo.mj_constraintUpdate(
        m,
        d,
        jar,
        !isnothing(cost) ? cost : C_NULL,
        flg_coneHessian,
    )

end
"""
    mj_getState(m, d, state, spec)

Get state.

# Arguments
- m::Model -> Constant.
- d::Data -> Constant.
- state::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- spec::Int32

# Constraints
- state should be a vector, not a matrix.
- state size should equal mj_stateSize(m, spec)

"""
function mj_getState(
    m,
    d,
    state::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    spec::Integer,
)
    if typeof(state) <: AbstractArray{Float64,2} && count(==(1), size(state)) < 1
        throw(ArgumentError("state should be a vector, not a matrix."))
    end

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return LibMuJoCo.mj_getState(m, d, state, spec)

end
"""
    mj_setState(m, d, state, spec)

Set state.

# Arguments
- m::Model -> Constant.
- d::Data
- state::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- spec::Int32

# Constraints
- state should be a vector, not a matrix.
- state size should equal mj_stateSize(m, spec)

"""
function mj_setState(
    m,
    d,
    state::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    spec::Integer,
)
    if typeof(state) <: AbstractArray{Float64,2} && count(==(1), size(state)) < 1
        throw(ArgumentError("state should be a vector, not a matrix."))
    end

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return LibMuJoCo.mj_setState(m, d, state, spec)

end
"""
    mj_mulJacVec(m, d, res, vec)

Multiply dense or sparse constraint Jacobian by vector.

# Arguments
- m::Model -> Constant.
- d::Data
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res should be of length nefc
- vec should be of length nv

"""
function mj_mulJacVec(
    m,
    d,
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != d.nefc)
        throw(ArgumentError("res should be of length nefc"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of length nv"))
    end
    return LibMuJoCo.mj_mulJacVec(m, d, res, vec)

end
"""
    mj_mulJacTVec(m, d, res, vec)

Multiply dense or sparse constraint Jacobian transpose by vector.

# Arguments
- m::Model -> Constant.
- d::Data
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res should be of length nv
- vec should be of length nefc

"""
function mj_mulJacTVec(
    m,
    d,
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of length nv"))
    end
    if (length(vec) != d.nefc)
        throw(ArgumentError("vec should be of length nefc"))
    end
    return LibMuJoCo.mj_mulJacTVec(m, d, res, vec)

end
"""
    mj_jac(m, d, jacp, jacr, point, body)

Compute 3/6-by-nv end-effector Jacobian of global point attached to given body.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- jacr::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- point::Vector{Float64} -> A vector of size 3. Constant.
- body::Int32

# Constraints
- point should be a vector of size 3
- point should be a vector of size 3.
- jacp should be of shape (3, nv)
- jacr should be of shape (3, nv)

"""
function mj_jac(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    jacr::Union{Nothing,AbstractArray{Float64,2}},
    point::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    body::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !isnothing(jacr) &&
       !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end
    if length(point) != 3
        throw(ArgumentError("point should be a vector of size 3"))
    end
    if typeof(point) <: AbstractArray{Float64,2} && count(==(1), size(point)) < 1
        throw(ArgumentError("point should be a vector of size 3."))
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jac(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        point[0+1],
        body,
    )

end
"""
    mj_jacBody(m, d, jacp, jacr, body)

Compute body frame end-effector Jacobian.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- jacr::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- body::Int32

# Constraints
- jacp should be of shape (3, nv)
- jacr should be of shape (3, nv)

"""
function mj_jacBody(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    jacr::Union{Nothing,AbstractArray{Float64,2}},
    body::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !isnothing(jacr) &&
       !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jacBody(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        body,
    )

end
"""
    mj_jacBodyCom(m, d, jacp, jacr, body)

Compute body center-of-mass end-effector Jacobian.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- jacr::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- body::Int32

# Constraints
- jacp should be of shape (3, nv)
- jacr should be of shape (3, nv)

"""
function mj_jacBodyCom(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    jacr::Union{Nothing,AbstractArray{Float64,2}},
    body::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !isnothing(jacr) &&
       !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jacBodyCom(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        body,
    )

end
"""
    mj_jacSubtreeCom(m, d, jacp, body)

Compute subtree center-of-mass end-effector Jacobian.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- body::Int32

# Constraints
- jacp should be of shape (3, nv)

"""
function mj_jacSubtreeCom(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    body::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jacSubtreeCom(m, d, !isnothing(jacp) ? jacp : C_NULL, body)

end
"""
    mj_jacGeom(m, d, jacp, jacr, geom)

Compute geom end-effector Jacobian.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- jacr::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- geom::Int32

# Constraints
- jacp should be of shape (3, nv)
- jacr should be of shape (3, nv)

"""
function mj_jacGeom(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    jacr::Union{Nothing,AbstractArray{Float64,2}},
    geom::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !isnothing(jacr) &&
       !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jacGeom(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        geom,
    )

end
"""
    mj_jacSite(m, d, jacp, jacr, site)

Compute site end-effector Jacobian.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- jacr::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- site::Int32

# Constraints
- jacp should be of shape (3, nv)
- jacr should be of shape (3, nv)

"""
function mj_jacSite(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    jacr::Union{Nothing,AbstractArray{Float64,2}},
    site::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !isnothing(jacr) &&
       !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jacSite(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        site,
    )

end
"""
    mj_jacPointAxis(m, d, jacp, jacr, point, axis, body)

Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.

# Arguments
- m::Model -> Constant.
- d::Data
- jacp::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- jacr::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- point::Vector{Float64} -> A vector of size 3. Constant.
- axis::Vector{Float64} -> A vector of size 3. Constant.
- body::Int32

# Constraints
- point should be a vector of size 3
- point should be a vector of size 3.
- axis should be a vector of size 3
- axis should be a vector of size 3.
- jacp should be of shape (3, nv)
- jacr should be of shape (3, nv)

"""
function mj_jacPointAxis(
    m,
    d,
    jacp::Union{Nothing,AbstractArray{Float64,2}},
    jacr::Union{Nothing,AbstractArray{Float64,2}},
    point::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    axis::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    body::Integer,
)
    if !isnothing(jacp) &&
       !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !isnothing(jacr) &&
       !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end
    if length(point) != 3
        throw(ArgumentError("point should be a vector of size 3"))
    end
    if typeof(point) <: AbstractArray{Float64,2} && count(==(1), size(point)) < 1
        throw(ArgumentError("point should be a vector of size 3."))
    end
    if length(axis) != 3
        throw(ArgumentError("axis should be a vector of size 3"))
    end
    if typeof(axis) <: AbstractArray{Float64,2} && count(==(1), size(axis)) < 1
        throw(ArgumentError("axis should be a vector of size 3."))
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return LibMuJoCo.mj_jacPointAxis(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        point[0+1],
        axis[0+1],
        body,
    )

end
"""
    mj_fullM(m, dst, M)

Convert sparse inertia matrix M into full (i.e. dense) matrix.

# Arguments
- m::Model -> Constant.
- dst::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- M::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- M should be a vector, not a matrix.
- M should be of size nM
- dst should be of shape (nv, nv)

"""
function mj_fullM(
    m,
    dst::AbstractArray{Float64,2},
    M::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !(typeof(dst) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("dst")
    end
    if typeof(M) <: AbstractArray{Float64,2} && count(==(1), size(M)) < 1
        throw(ArgumentError("M should be a vector, not a matrix."))
    end

    if (length(M) != m.nM)
        throw(ArgumentError("M should be of size nM"))
    end
    if (size(dst, 2) != m.nv || size(dst, 1) != m.nv)
        throw(ArgumentError("dst should be of shape (nv, nv)"))
    end
    return LibMuJoCo.mj_fullM(m, dst, M)

end
"""
    mj_mulM(m, d, res, vec)

Multiply vector by inertia matrix.

# Arguments
- m::Model -> Constant.
- d::Data -> Constant.
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res should be of size nv
- vec should be of size nv

"""
function mj_mulM(
    m,
    d,
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of size nv"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of size nv"))
    end
    return LibMuJoCo.mj_mulM(m, d, res, vec)

end
"""
    mj_mulM2(m, d, res, vec)

Multiply vector by (inertia matrix)^(1/2).

# Arguments
- m::Model -> Constant.
- d::Data -> Constant.
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res should be of size nv
- vec should be of size nv

"""
function mj_mulM2(
    m,
    d,
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of size nv"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of size nv"))
    end
    return LibMuJoCo.mj_mulM2(m, d, res, vec)

end
"""
    mj_addM(m, d, dst, rownnz, rowadr, colind)

Add inertia matrix to destination matrix. Destination can be sparse uncompressed, or dense when all int* are NULL

# Arguments
- m::Model -> Constant.
- d::Data
- dst::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- rownnz::Vector{Int32} -> A vector of variable size. Check constraints for sizes.
- rowadr::Vector{Int32} -> A vector of variable size. Check constraints for sizes.
- colind::Vector{Int32} -> A vector of variable size. Check constraints for sizes.

# Constraints
- dst should be a vector, not a matrix.
- rownnz should be a vector, not a matrix.
- rowadr should be a vector, not a matrix.
- colind should be a vector, not a matrix.
- dst should be of size nM
- rownnz should be of size nv
- rowadr should be of size nv
- colind should be of size nM

"""
function mj_addM(
    m,
    d,
    dst::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    rownnz::Union{AbstractVector{Int32},AbstractArray{Int32,2}},
    rowadr::Union{AbstractVector{Int32},AbstractArray{Int32,2}},
    colind::Union{AbstractVector{Int32},AbstractArray{Int32,2}},
)
    if typeof(dst) <: AbstractArray{Float64,2} && count(==(1), size(dst)) < 1
        throw(ArgumentError("dst should be a vector, not a matrix."))
    end
    if typeof(rownnz) <: AbstractArray{Int32,2} && count(==(1), size(rownnz)) < 1
        throw(ArgumentError("rownnz should be a vector, not a matrix."))
    end
    if typeof(rowadr) <: AbstractArray{Int32,2} && count(==(1), size(rowadr)) < 1
        throw(ArgumentError("rowadr should be a vector, not a matrix."))
    end
    if typeof(colind) <: AbstractArray{Int32,2} && count(==(1), size(colind)) < 1
        throw(ArgumentError("colind should be a vector, not a matrix."))
    end

    if (length(dst) != m.nM)
        throw(ArgumentError("dst should be of size nM"))
    end
    if (length(rownnz) != m.nv)
        throw(ArgumentError("rownnz should be of size nv"))
    end
    if (length(rowadr) != m.nv)
        throw(ArgumentError("rowadr should be of size nv"))
    end
    if (length(colind) != m.nM)
        throw(ArgumentError("colind should be of size nM"))
    end
    return LibMuJoCo.mj_addM(m, d, dst, rownnz, rowadr, colind)

end
"""
    mj_applyFT(m, d, force, torque, point, body, qfrc_target)

Apply Cartesian force and torque (outside xfrc_applied mechanism).

# Arguments
- m::Model -> Constant.
- d::Data
- force::Vector{Float64} -> A vector of size 3. Constant.
- torque::Vector{Float64} -> A vector of size 3. Constant.
- point::Vector{Float64} -> A vector of size 3. Constant.
- body::Int32
- qfrc_target::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- force should be a vector of size 3
- force should be a vector of size 3.
- torque should be a vector of size 3
- torque should be a vector of size 3.
- point should be a vector of size 3
- point should be a vector of size 3.
- qfrc_target should be a vector, not a matrix.
- qfrc_target should be of size nv

"""
function mj_applyFT(
    m,
    d,
    force::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    torque::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    point::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    body::Integer,
    qfrc_target::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if length(force) != 3
        throw(ArgumentError("force should be a vector of size 3"))
    end
    if typeof(force) <: AbstractArray{Float64,2} && count(==(1), size(force)) < 1
        throw(ArgumentError("force should be a vector of size 3."))
    end
    if length(torque) != 3
        throw(ArgumentError("torque should be a vector of size 3"))
    end
    if typeof(torque) <: AbstractArray{Float64,2} && count(==(1), size(torque)) < 1
        throw(ArgumentError("torque should be a vector of size 3."))
    end
    if length(point) != 3
        throw(ArgumentError("point should be a vector of size 3"))
    end
    if typeof(point) <: AbstractArray{Float64,2} && count(==(1), size(point)) < 1
        throw(ArgumentError("point should be a vector of size 3."))
    end
    if typeof(qfrc_target) <: AbstractArray{Float64,2} &&
       count(==(1), size(qfrc_target)) < 1
        throw(ArgumentError("qfrc_target should be a vector, not a matrix."))
    end

    if (length(qfrc_target) != m.nv)
        throw(ArgumentError("qfrc_target should be of size nv"))
    end
    return LibMuJoCo.mj_applyFT(
        m,
        d,
        force[0+1],
        torque[0+1],
        point[0+1],
        body,
        qfrc_target,
    )

end
"""
    mj_differentiatePos(m, qvel, dt, qpos1, qpos2)

Compute velocity by finite-differencing two positions.

# Arguments
- m::Model -> Constant.
- qvel::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- dt::Float64
- qpos1::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- qpos2::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- qvel should be a vector, not a matrix.
- qpos1 should be a vector, not a matrix.
- qpos2 should be a vector, not a matrix.
- qvel should be of size nv
- qpos1 should be of size nq
- qpos2 should be of size nq

"""
function mj_differentiatePos(
    m,
    qvel::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    dt::Number,
    qpos1::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    qpos2::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(qvel) <: AbstractArray{Float64,2} && count(==(1), size(qvel)) < 1
        throw(ArgumentError("qvel should be a vector, not a matrix."))
    end
    if typeof(qpos1) <: AbstractArray{Float64,2} && count(==(1), size(qpos1)) < 1
        throw(ArgumentError("qpos1 should be a vector, not a matrix."))
    end
    if typeof(qpos2) <: AbstractArray{Float64,2} && count(==(1), size(qpos2)) < 1
        throw(ArgumentError("qpos2 should be a vector, not a matrix."))
    end

    if (length(qvel) != m.nv)
        throw(ArgumentError("qvel should be of size nv"))
    end
    if (length(qpos1) != m.nq)
        throw(ArgumentError("qpos1 should be of size nq"))
    end
    if (length(qpos2) != m.nq)
        throw(ArgumentError("qpos2 should be of size nq"))
    end
    return LibMuJoCo.mj_differentiatePos(m, qvel, dt, qpos1, qpos2)

end
"""
    mj_integratePos(m, qpos, qvel, dt)

Integrate position with given velocity.

# Arguments
- m::Model -> Constant.
- qpos::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- qvel::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- dt::Float64

# Constraints
- qpos should be a vector, not a matrix.
- qvel should be a vector, not a matrix.
- qpos should be of size nq
- qvel should be of size nv

"""
function mj_integratePos(
    m,
    qpos::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    qvel::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    dt::Number,
)
    if typeof(qpos) <: AbstractArray{Float64,2} && count(==(1), size(qpos)) < 1
        throw(ArgumentError("qpos should be a vector, not a matrix."))
    end
    if typeof(qvel) <: AbstractArray{Float64,2} && count(==(1), size(qvel)) < 1
        throw(ArgumentError("qvel should be a vector, not a matrix."))
    end

    if (length(qpos) != m.nq)
        throw(ArgumentError("qpos should be of size nq"))
    end
    if (length(qvel) != m.nv)
        throw(ArgumentError("qvel should be of size nv"))
    end
    return LibMuJoCo.mj_integratePos(m, qpos, qvel, dt)

end
"""
    mj_normalizeQuat(m, qpos)

Normalize all quaternions in qpos-type vector.

# Arguments
- m::Model -> Constant.
- qpos::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- qpos should be a vector, not a matrix.
- qpos should be of size nq

"""
function mj_normalizeQuat(m, qpos::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(qpos) <: AbstractArray{Float64,2} && count(==(1), size(qpos)) < 1
        throw(ArgumentError("qpos should be a vector, not a matrix."))
    end

    if (length(qpos) != m.nq)
        throw(ArgumentError("qpos should be of size nq"))
    end
    return LibMuJoCo.mj_normalizeQuat(m, qpos)

end
"""
    mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)

Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude. Return distance (x) to nearest surface, or -1 if no intersection and output geomid. geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.

# Arguments
- m::Model -> Constant.
- d::Data -> Constant.
- pnt::Vector{Float64} -> A vector of size 3. Constant.
- vec::Vector{Float64} -> A vector of size 3. Constant.
- geomgroup::Vector{UInt8} -> An optional vector of size 6. Constant.
- flg_static::UInt8
- bodyexclude::Int32
- geomid::Vector{Int32} -> A vector of size 1.

# Constraints
- pnt should be a vector of size 3
- pnt should be a vector of size 3.
- vec should be a vector of size 3
- vec should be a vector of size 3.
- geomgroup should be a vector of size 6
- geomgroup should be a vector of size 6.
- geomid should be a vector of size 1
- geomid should be a vector of size 1.

"""
function mj_ray(
    m,
    d,
    pnt::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    geomgroup::Union{Nothing,AbstractVector{UInt8},AbstractArray{UInt8,2}},
    flg_static::UInt8,
    bodyexclude::Integer,
    geomid::Union{AbstractVector{Int32},AbstractArray{Int32,2}},
)
    if length(pnt) != 3
        throw(ArgumentError("pnt should be a vector of size 3"))
    end
    if typeof(pnt) <: AbstractArray{Float64,2} && count(==(1), size(pnt)) < 1
        throw(ArgumentError("pnt should be a vector of size 3."))
    end
    if length(vec) != 3
        throw(ArgumentError("vec should be a vector of size 3"))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector of size 3."))
    end
    if !isnothing(geomgroup) && length(geomgroup) != 6
        throw(ArgumentError("geomgroup should be a vector of size 6"))
    end
    if !isnothing(geomgroup) &&
       typeof(geomgroup) <: AbstractArray{UInt8,2} &&
       count(==(1), size(geomgroup)) < 1
        throw(ArgumentError("geomgroup should be a vector of size 6."))
    end
    if length(geomid) != 1
        throw(ArgumentError("geomid should be a vector of size 1"))
    end
    if typeof(geomid) <: AbstractArray{Int32,2} && count(==(1), size(geomid)) < 1
        throw(ArgumentError("geomid should be a vector of size 1."))
    end

    return LibMuJoCo.mj_ray(
        m,
        d,
        pnt[0+1],
        vec[0+1],
        !isnothing(geomgroup) ? geomgroup : C_NULL,
        flg_static,
        bodyexclude,
        geomid[0+1],
    )

end
"""
    mju_zero(res)

Set res = 0.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- res should be a vector, not a matrix.

"""
function mju_zero(res::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_zero(res, length(res))

end
"""
    mju_fill(res, val)

Set res = val.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- val::Float64

# Constraints
- res should be a vector, not a matrix.

"""
function mju_fill(res::Union{AbstractVector{Float64},AbstractArray{Float64,2}}, val::Number)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_fill(res, val, length(res))

end
"""
    mju_copy(res, data)

Set res = vec.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- data::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- data should be a vector, not a matrix.
- res and data should have the same size

"""
function mju_copy(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    data::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(data) <: AbstractArray{Float64,2} && count(==(1), size(data)) < 1
        throw(ArgumentError("data should be a vector, not a matrix."))
    end

    if (length(res) != length(data))
        throw(ArgumentError("res and data should have the same size"))
    end
    return LibMuJoCo.mju_copy(res, data, length(res))

end
"""
    mju_sum(vec)

Return sum(vec).

# Arguments
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- vec should be a vector, not a matrix.

"""
function mju_sum(vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_sum(vec, length(vec))

end
"""
    mju_L1(vec)

Return L1 norm: sum(abs(vec)).

# Arguments
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- vec should be a vector, not a matrix.

"""
function mju_L1(vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_L1(vec, length(vec))

end
"""
    mju_scl(res, vec, scl)

Set res = vec*scl.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- scl::Float64

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_scl(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    scl::Number,
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_scl(res, vec, scl, length(res))

end
"""
    mju_add(res, vec1, vec2)

Set res = vec1 + vec2.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec1::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- vec2::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec1 should be a vector, not a matrix.
- vec2 should be a vector, not a matrix.
- res and vec1 should have the same size
- res and vec2 should have the same size

"""
function mju_add(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec1::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec1) <: AbstractArray{Float64,2} && count(==(1), size(vec1)) < 1
        throw(ArgumentError("vec1 should be a vector, not a matrix."))
    end
    if typeof(vec2) <: AbstractArray{Float64,2} && count(==(1), size(vec2)) < 1
        throw(ArgumentError("vec2 should be a vector, not a matrix."))
    end

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return LibMuJoCo.mju_add(res, vec1, vec2, length(res))

end
"""
    mju_sub(res, vec1, vec2)

Set res = vec1 - vec2.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec1::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- vec2::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec1 should be a vector, not a matrix.
- vec2 should be a vector, not a matrix.
- res and vec1 should have the same size
- res and vec2 should have the same size

"""
function mju_sub(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec1::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec1) <: AbstractArray{Float64,2} && count(==(1), size(vec1)) < 1
        throw(ArgumentError("vec1 should be a vector, not a matrix."))
    end
    if typeof(vec2) <: AbstractArray{Float64,2} && count(==(1), size(vec2)) < 1
        throw(ArgumentError("vec2 should be a vector, not a matrix."))
    end

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return LibMuJoCo.mju_sub(res, vec1, vec2, length(res))

end
"""
    mju_addTo(res, vec)

Set res = res + vec.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_addTo(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_addTo(res, vec, length(res))

end
"""
    mju_subFrom(res, vec)

Set res = res - vec.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_subFrom(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_subFrom(res, vec, length(res))

end
"""
    mju_addToScl(res, vec, scl)

Set res = res + vec*scl.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- scl::Float64

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_addToScl(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    scl::Number,
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_addToScl(res, vec, scl, length(res))

end
"""
    mju_addScl(res, vec1, vec2, scl)

Set res = vec1 + vec2*scl.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec1::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- vec2::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- scl::Float64

# Constraints
- res should be a vector, not a matrix.
- vec1 should be a vector, not a matrix.
- vec2 should be a vector, not a matrix.
- res and vec1 should have the same size
- res and vec2 should have the same size

"""
function mju_addScl(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec1::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    scl::Number,
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec1) <: AbstractArray{Float64,2} && count(==(1), size(vec1)) < 1
        throw(ArgumentError("vec1 should be a vector, not a matrix."))
    end
    if typeof(vec2) <: AbstractArray{Float64,2} && count(==(1), size(vec2)) < 1
        throw(ArgumentError("vec2 should be a vector, not a matrix."))
    end

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return LibMuJoCo.mju_addScl(res, vec1, vec2, scl, length(res))

end
"""
    mju_normalize(vec)

Normalize vector, return length before normalization.

# Arguments
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- vec should be a vector, not a matrix.

"""
function mju_normalize(vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_normalize(vec, length(vec))

end
"""
    mju_norm(vec)

Return vector length (without normalizing vector).

# Arguments
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- vec should be a vector, not a matrix.

"""
function mju_norm(vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_norm(vec, length(vec))

end
"""
    mju_dot(vec1, vec2)

Return dot-product of vec1 and vec2.

# Arguments
- vec1::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- vec2::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- vec1 should be a vector, not a matrix.
- vec2 should be a vector, not a matrix.
- vec1 and vec2 should have the same size

"""
function mju_dot(
    vec1::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(vec1) <: AbstractArray{Float64,2} && count(==(1), size(vec1)) < 1
        throw(ArgumentError("vec1 should be a vector, not a matrix."))
    end
    if typeof(vec2) <: AbstractArray{Float64,2} && count(==(1), size(vec2)) < 1
        throw(ArgumentError("vec2 should be a vector, not a matrix."))
    end

    if (length(vec1) != length(vec2))
        throw(ArgumentError("vec1 and vec2 should have the same size"))
    end
    return LibMuJoCo.mju_dot(vec1, vec2, length(vec1))

end
"""
    mju_mulMatVec(res, mat, vec)

Multiply matrix and vector: res = mat * vec.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.

"""
function mju_mulMatVec(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::AbstractArray{Float64,2},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != size(mat, 1))
        throw(ArgumentError("size of res should equal the number of rows in mat"))
    end
    if (length(vec) != size(mat, 2))
        throw(ArgumentError("size of vec should equal the number of columns in mat"))
    end
    return LibMuJoCo.mju_mulMatVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
"""
    mju_mulMatTVec(res, mat, vec)

Multiply transposed matrix and vector: res = mat' * vec.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.

"""
function mju_mulMatTVec(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::AbstractArray{Float64,2},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != size(mat, 2))
        throw(ArgumentError("size of res should equal the number of columns in mat"))
    end
    if (length(vec) != size(mat, 1))
        throw(ArgumentError("size of vec should equal the number of rows in mat"))
    end
    return LibMuJoCo.mju_mulMatTVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
"""
    mju_mulVecMatVec(vec1, mat, vec2)

Multiply square matrix with vectors on both sides: returns vec1' * mat * vec2.

# Arguments
- vec1::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- vec2::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- vec1 should be a vector, not a matrix.
- vec2 should be a vector, not a matrix.

"""
function mju_mulVecMatVec(
    vec1::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::AbstractArray{Float64,2},
    vec2::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(vec1) <: AbstractArray{Float64,2} && count(==(1), size(vec1)) < 1
        throw(ArgumentError("vec1 should be a vector, not a matrix."))
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if typeof(vec2) <: AbstractArray{Float64,2} && count(==(1), size(vec2)) < 1
        throw(ArgumentError("vec2 should be a vector, not a matrix."))
    end

    if (length(vec1) != length(vec2))
        throw(ArgumentError("size of vec1 should equal the size of vec2"))
    end
    if (length(vec1) != size(mat, 2))
        throw(ArgumentError("size of vectors should equal the number of columns in mat"))
    end
    if (length(vec1) != size(mat, 1))
        throw(ArgumentError("size of vectors should equal the number of rows in mat"))
    end
    return LibMuJoCo.mju_mulVecMatVec(vec1, mat, vec2, length(vec1))

end
"""
    mju_transpose(res, mat)

Transpose matrix: res = mat'.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.

# Constraints
- #columns in res should equal #rows in mat
- #rows in res should equal #columns in mat

"""
function mju_transpose(res::AbstractArray{Float64,2}, mat::AbstractArray{Float64,2})
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(res, 2) != size(mat, 1))
        throw(ArgumentError("#columns in res should equal #rows in mat"))
    end
    if (size(res, 1) != size(mat, 2))
        throw(ArgumentError("#rows in res should equal #columns in mat"))
    end
    return LibMuJoCo.mju_transpose(res, mat, size(mat, 1), size(mat, 2))

end
"""
    mju_symmetrize(res, mat)

Symmetrize square matrix res = (mat + mat')/2.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.

# Constraints
- mat should be square
- res and mat should have the same shape

"""
function mju_symmetrize(res::AbstractArray{Float64,2}, mat::AbstractArray{Float64,2})
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 2) != size(mat, 1))
        throw(ArgumentError("mat should be square"))
    end
    if (size(res, 2) != size(mat, 2) || size(res, 1) != size(mat, 1))
        throw(ArgumentError("res and mat should have the same shape"))
    end
    return LibMuJoCo.mju_symmetrize(res, mat, size(mat, 1))

end
"""
    mju_eye(mat)

Set mat to the identity matrix.

# Arguments
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.

# Constraints
- mat should be square

"""
function mju_eye(mat::AbstractArray{Float64,2})
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 2) != size(mat, 1))
        throw(ArgumentError("mat should be square"))
    end
    return LibMuJoCo.mju_eye(mat, size(mat, 1))

end
"""
    mju_mulMatMat(res, mat1, mat2)

Multiply matrices: res = mat1 * mat2.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat1::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- mat2::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.

# Constraints
- #rows in res should equal #rows in mat1
- #columns in mat1 should equal #rows in mat2

"""
function mju_mulMatMat(
    res::AbstractArray{Float64,2},
    mat1::AbstractArray{Float64,2},
    mat2::AbstractArray{Float64,2},
)
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !(typeof(mat1) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat1")
    end
    if !(typeof(mat2) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat2")
    end

    if (size(res, 1) != size(mat1, 1))
        throw(ArgumentError("#rows in res should equal #rows in mat1"))
    end
    if (size(res, 2) != size(mat2, 2))
        throw(ArgumentError("#columns in res should equal #columns in mat2"))
    end
    if (size(mat1, 2) != size(mat2, 1))
        throw(ArgumentError("#columns in mat1 should equal #rows in mat2"))
    end
    return LibMuJoCo.mju_mulMatMat(
        res,
        mat1,
        mat2,
        size(mat1, 1),
        size(mat1, 2),
        size(mat2, 2),
    )

end
"""
    mju_mulMatMatT(res, mat1, mat2)

Multiply matrices, second argument transposed: res = mat1 * mat2'.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat1::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- mat2::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.

# Constraints
- #rows in res should equal #rows in mat1
- #columns in res should equal #rows in mat2

"""
function mju_mulMatMatT(
    res::AbstractArray{Float64,2},
    mat1::AbstractArray{Float64,2},
    mat2::AbstractArray{Float64,2},
)
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !(typeof(mat1) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat1")
    end
    if !(typeof(mat2) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat2")
    end

    if (size(res, 1) != size(mat1, 1))
        throw(ArgumentError("#rows in res should equal #rows in mat1"))
    end
    if (size(res, 2) != size(mat2, 1))
        throw(ArgumentError("#columns in res should equal #rows in mat2"))
    end
    if (size(mat1, 2) != size(mat2, 2))
        throw(ArgumentError("#columns in mat1 should equal #columns in mat2"))
    end
    return LibMuJoCo.mju_mulMatMatT(
        res,
        mat1,
        mat2,
        size(mat1, 1),
        size(mat1, 2),
        size(mat2, 1),
    )

end
"""
    mju_mulMatTMat(res, mat1, mat2)

Multiply matrices, first argument transposed: res = mat1' * mat2.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat1::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- mat2::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.

# Constraints
- #rows in res should equal #columns in mat1
- #rows in mat1 should equal #rows in mat2

"""
function mju_mulMatTMat(
    res::AbstractArray{Float64,2},
    mat1::AbstractArray{Float64,2},
    mat2::AbstractArray{Float64,2},
)
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !(typeof(mat1) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat1")
    end
    if !(typeof(mat2) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat2")
    end

    if (size(res, 1) != size(mat1, 2))
        throw(ArgumentError("#rows in res should equal #columns in mat1"))
    end
    if (size(res, 2) != size(mat2, 2))
        throw(ArgumentError("#columns in res should equal #columns in mat2"))
    end
    if (size(mat1, 1) != size(mat2, 1))
        throw(ArgumentError("#rows in mat1 should equal #rows in mat2"))
    end
    return LibMuJoCo.mju_mulMatTMat(
        res,
        mat1,
        mat2,
        size(mat1, 1),
        size(mat1, 2),
        size(mat2, 2),
    )

end
"""
    mju_sqrMatTD(res, mat, diag)

Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- diag::Vector{Float64} -> An optional vector of variable size. Check constraints for sizes.

# Constraints
- diag should be a vector, not a matrix.
- #rows in res should equal #columns in mat
- #rows in res should equal #columns in mat

"""
function mju_sqrMatTD(
    res::AbstractArray{Float64,2},
    mat::AbstractArray{Float64,2},
    diag::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(diag) &&
       typeof(diag) <: AbstractArray{Float64,2} &&
       count(==(1), size(diag)) < 1
        throw(ArgumentError("diag should be a vector, not a matrix."))
    end

    if (size(res, 1) != size(mat, 2))
        throw(ArgumentError("#rows in res should equal #columns in mat"))
    end
    if (size(res, 2) != size(mat, 2))
        throw(ArgumentError("#rows in res should equal #columns in mat"))
    end
    if (!isnothing(diag) && length(diag) != size(mat, 1))
        throw(ArgumentError("size of diag should equal the number of rows in mat"))
    end
    return LibMuJoCo.mju_sqrMatTD(
        res,
        mat,
        !isnothing(diag) ? diag : C_NULL,
        size(mat, 1),
        size(mat, 2),
    )

end
"""
    mju_cholFactor(mat, mindiag)

Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.

# Arguments
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mindiag::Float64

# Constraints
- mat should be a square matrix

"""
function mju_cholFactor(mat::AbstractArray{Float64,2}, mindiag::Number)
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    return LibMuJoCo.mju_cholFactor(mat, size(mat, 1), mindiag)

end
"""
    mju_cholSolve(res, mat, vec)

Solve (mat*mat') * res = vec, where mat is a Cholesky factor.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- mat should be a square matrix

"""
function mju_cholSolve(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::AbstractArray{Float64,2},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    if (length(res) != size(mat, 1))
        throw(ArgumentError("size of res should equal the number of rows in mat"))
    end
    if (length(vec) != size(mat, 2))
        throw(ArgumentError("size of vec should equal the number of rows in mat"))
    end
    return LibMuJoCo.mju_cholSolve(res, mat, vec, size(mat, 1))

end
"""
    mju_cholUpdate(mat, x, flg_plus)

Cholesky rank-one update: L*L' +/- x*x'; return rank.

# Arguments
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- x::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- flg_plus::Int32

# Constraints
- x should be a vector, not a matrix.
- mat should be a square matrix

"""
function mju_cholUpdate(
    mat::AbstractArray{Float64,2},
    x::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    flg_plus::Integer,
)
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if typeof(x) <: AbstractArray{Float64,2} && count(==(1), size(x)) < 1
        throw(ArgumentError("x should be a vector, not a matrix."))
    end

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    if (length(x) != size(mat, 1))
        throw(ArgumentError("size of x should equal the number of rows in mat"))
    end
    return LibMuJoCo.mju_cholUpdate(mat, x, size(mat, 1), flg_plus)

end
"""
    mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)

Band-dense Cholesky decomposition.  Returns minimum value in the factorized diagonal, or 0 if rank-deficient.  mat has (ntotal-ndense) x nband + ndense x ntotal elements.  The first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive.  The second ndense x ntotal store the band part as entire dense rows.  Add diagadd+diagmul*mat_ii to diagonal before factorization.

# Arguments
- mat::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- ntotal::Int32
- nband::Int32
- ndense::Int32
- diagadd::Float64
- diagmul::Float64

# Constraints
- mat should be a vector, not a matrix.

"""
function mju_cholFactorBand(
    mat::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
    diagadd::Number,
    diagmul::Number,
)
    if typeof(mat) <: AbstractArray{Float64,2} && count(==(1), size(mat)) < 1
        throw(ArgumentError("mat should be a vector, not a matrix."))
    end

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(ArgumentError("mat must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    return LibMuJoCo.mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)

end
"""
    mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)

Solve (mat*mat')*res = vec where mat is a band-dense Cholesky factor.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- mat::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- ntotal::Int32
- nband::Int32
- ndense::Int32

# Constraints
- res should be a vector, not a matrix.
- mat should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- size of res should equal ntotal
- size of vec should equal ntotal

"""
function mju_cholSolveBand(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(mat) <: AbstractArray{Float64,2} && count(==(1), size(mat)) < 1
        throw(ArgumentError("mat should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(
            ArgumentError("mat must have (ntotal-ndense)*nband +  ndense*ntotal elements"),
        )
    end
    if (length(res) != ntotal)
        throw(ArgumentError("size of res should equal ntotal"))
    end
    if (length(vec) != ntotal)
        throw(ArgumentError("size of vec should equal ntotal"))
    end
    return LibMuJoCo.mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)

end
"""
    mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)

Convert banded matrix to dense matrix, fill upper triangle if flg_sym>0.

# Arguments
- res::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- mat::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- ntotal::Int32
- nband::Int32
- ndense::Int32
- flg_sym::UInt8

# Constraints
- mat should be a vector, not a matrix.
- res should have ntotal rows
- res should have ntotal columns

"""
function mju_band2Dense(
    res::AbstractArray{Float64,2},
    mat::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
    flg_sym::UInt8,
)
    if !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if typeof(mat) <: AbstractArray{Float64,2} && count(==(1), size(mat)) < 1
        throw(ArgumentError("mat should be a vector, not a matrix."))
    end

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(ArgumentError("mat must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    if (size(res, 1) != ntotal)
        throw(ArgumentError("res should have ntotal rows"))
    end
    if (size(res, 2) != ntotal)
        throw(ArgumentError("res should have ntotal columns"))
    end
    return LibMuJoCo.mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)

end
"""
    mju_dense2Band(res, mat, ntotal, nband, ndense)

Convert dense matrix to banded matrix.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- ntotal::Int32
- nband::Int32
- ndense::Int32

# Constraints
- res should be a vector, not a matrix.
- mat should have ntotal rows
- mat should have ntotal columns

"""
function mju_dense2Band(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::AbstractArray{Float64,2},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    nRes = (ntotal - ndense) * nband + ndense * ntotal
    if (length(res) != nRes)
        throw(ArgumentError("res must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    if (size(mat, 1) != ntotal)
        throw(ArgumentError("mat should have ntotal rows"))
    end
    if (size(mat, 2) != ntotal)
        throw(ArgumentError("mat should have ntotal columns"))
    end
    return LibMuJoCo.mju_dense2Band(res, mat, ntotal, nband, ndense)

end
"""
    mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nVec, flg_sym)

Multiply band-diagonal matrix with nvec vectors, include upper triangle if flg_sym>0.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- mat::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- vec::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- ntotal::Int32
- nband::Int32
- ndense::Int32
- nVec::Int32
- flg_sym::UInt8

# Constraints
- res should be a vector, not a matrix.
- res should have ntotal rows
- res should have nVec columns
- vec should have ntotal rows
- vec should have nVec columns

"""
function mju_bandMulMatVec(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::AbstractArray{Float64,2},
    vec::AbstractArray{Float64,2},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
    nVec::Integer,
    flg_sym::UInt8,
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !(typeof(vec) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("vec")
    end

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(ArgumentError("mat must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    if (size(res, 1) != ntotal)
        throw(ArgumentError("res should have ntotal rows"))
    end
    if (size(res, 2) != nVec)
        throw(ArgumentError("res should have nVec columns"))
    end
    if (size(vec, 1) != ntotal)
        throw(ArgumentError("vec should have ntotal rows"))
    end
    if (size(vec, 2) != nVec)
        throw(ArgumentError("vec should have nVec columns"))
    end
    return LibMuJoCo.mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nVec, flg_sym)

end
"""
    mju_boxQP(res, R, index, H, g, lower, upper)

minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <= upper, return rank or -1 if failed   inputs:     n           - problem dimension     H           - SPD matrix                n*n     g           - bias vector               n     lower       - lower bounds              n     upper       - upper bounds              n     res         - solution warmstart        n   return value:     nfree <= n  - rank of unconstrained subspace, -1 if failure   outputs (required):     res         - solution                  n     R           - subspace Cholesky factor  nfree*nfree    allocated: n*(n+7)   outputs (optional):     index       - set of free dimensions    nfree          allocated: n   notes:     the initial value of res is used to warmstart the solver     R must have allocatd size n*(n+7), but only nfree*nfree values are used in output     index (if given) must have allocated size n, but only nfree values are used in output     only the lower triangles of H and R and are read from and written to, respectively     the convenience function mju_boxQPmalloc allocates the required data structures

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- R::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- index::Vector{Int32} -> An optional vector of variable size. Check constraints for sizes.
- H::Matrix{Float64} -> A matrix variable size. Check constraints for sizes. Constant.
- g::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- lower::Vector{Float64} -> An optional vector of variable size. Check constraints for sizes. Constant.
- upper::Vector{Float64} -> An optional vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- index should be a vector, not a matrix.
- g should be a vector, not a matrix.
- lower should be a vector, not a matrix.
- upper should be a vector, not a matrix.
- size of R should be n*(n+7)
- size of index should equal n
- H should be of shape (n, n)
- size of g should equal n
- size of lower should equal n
- size of upper should equal n

"""
function mju_boxQP(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    R::AbstractArray{Float64,2},
    index::Union{Nothing,AbstractVector{Int32},AbstractArray{Int32,2}},
    H::AbstractArray{Float64,2},
    g::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    lower::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    upper::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if !(typeof(R) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("R")
    end
    if !isnothing(index) &&
       typeof(index) <: AbstractArray{Int32,2} &&
       count(==(1), size(index)) < 1
        throw(ArgumentError("index should be a vector, not a matrix."))
    end
    if !(typeof(H) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("H")
    end
    if typeof(g) <: AbstractArray{Float64,2} && count(==(1), size(g)) < 1
        throw(ArgumentError("g should be a vector, not a matrix."))
    end
    if !isnothing(lower) &&
       typeof(lower) <: AbstractArray{Float64,2} &&
       count(==(1), size(lower)) < 1
        throw(ArgumentError("lower should be a vector, not a matrix."))
    end
    if !isnothing(upper) &&
       typeof(upper) <: AbstractArray{Float64,2} &&
       count(==(1), size(upper)) < 1
        throw(ArgumentError("upper should be a vector, not a matrix."))
    end

    n = length(res)
    if (length(R) != n * (n + 7))
        throw(ArgumentError("size of R should be n*(n+7)"))
    end
    if (!isnothing(index) && (length(index) != n))
        throw(ArgumentError("size of index should equal n"))
    end
    if (size(H, 1) != n || size(H, 2) != n)
        throw(ArgumentError("H should be of shape (n, n)"))
    end
    if (length(g) != n)
        throw(ArgumentError("size of g should equal n"))
    end
    if (!isnothing(lower) && (length(lower) != n))
        throw(ArgumentError("size of lower should equal n"))
    end
    if (!isnothing(upper) && (length(upper) != n))
        throw(ArgumentError("size of upper should equal n"))
    end
    return LibMuJoCo.mju_boxQP(
        res,
        R,
        !isnothing(index) ? index : C_NULL,
        H,
        g,
        n,
        !isnothing(lower) ? lower : C_NULL,
        !isnothing(upper) ? upper : C_NULL,
    )

end
"""
    mju_encodePyramid(pyramid, force, mu)

Convert contact force to pyramid representation.

# Arguments
- pyramid::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- force::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- mu::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- pyramid should be a vector, not a matrix.
- force should be a vector, not a matrix.
- mu should be a vector, not a matrix.

"""
function mju_encodePyramid(
    pyramid::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    force::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mu::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(pyramid) <: AbstractArray{Float64,2} && count(==(1), size(pyramid)) < 1
        throw(ArgumentError("pyramid should be a vector, not a matrix."))
    end
    if typeof(force) <: AbstractArray{Float64,2} && count(==(1), size(force)) < 1
        throw(ArgumentError("force should be a vector, not a matrix."))
    end
    if typeof(mu) <: AbstractArray{Float64,2} && count(==(1), size(mu)) < 1
        throw(ArgumentError("mu should be a vector, not a matrix."))
    end

    if (length(pyramid) != 2 * length(mu))
        throw(ArgumentError("size of pyramid should be twice as large as size of mu"))
    end
    if (length(force) != length(mu) + 1)
        throw(ArgumentError("size of force should be exactly one larger than size of mu"))
    end
    return LibMuJoCo.mju_encodePyramid(pyramid, force, mu, length(mu))

end
"""
    mju_decodePyramid(force, pyramid, mu)

Convert pyramid representation to contact force.

# Arguments
- force::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- pyramid::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.
- mu::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- force should be a vector, not a matrix.
- pyramid should be a vector, not a matrix.
- mu should be a vector, not a matrix.

"""
function mju_decodePyramid(
    force::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    pyramid::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    mu::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(force) <: AbstractArray{Float64,2} && count(==(1), size(force)) < 1
        throw(ArgumentError("force should be a vector, not a matrix."))
    end
    if typeof(pyramid) <: AbstractArray{Float64,2} && count(==(1), size(pyramid)) < 1
        throw(ArgumentError("pyramid should be a vector, not a matrix."))
    end
    if typeof(mu) <: AbstractArray{Float64,2} && count(==(1), size(mu)) < 1
        throw(ArgumentError("mu should be a vector, not a matrix."))
    end

    if (length(pyramid) != 2 * length(mu))
        throw(ArgumentError("size of pyramid should be twice as large as size of mu"))
    end
    if (length(force) != length(mu) + 1)
        throw(ArgumentError("size of force should be exactly one larger than size of mu"))
    end
    return LibMuJoCo.mju_decodePyramid(force, pyramid, mu, length(mu))

end
"""
    mju_isZero(vec)

Return 1 if all elements are 0.

# Arguments
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- vec should be a vector, not a matrix.

"""
function mju_isZero(vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_isZero(vec, length(vec))

end
"""
    mju_f2n(res, vec)

Convert from float to mjtNum.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float32} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_f2n(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float32},AbstractArray{Float32,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float32,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_f2n(res, vec, length(res))

end
"""
    mju_n2f(res, vec)

Convert from mjtNum to float.

# Arguments
- res::Vector{Float32} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_n2f(
    res::Union{AbstractVector{Float32},AbstractArray{Float32,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float32,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_n2f(res, vec, length(res))

end
"""
    mju_d2n(res, vec)

Convert from double to mjtNum.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_d2n(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_d2n(res, vec, length(res))

end
"""
    mju_n2d(res, vec)

Convert from mjtNum to double.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.
- vec::Vector{Float64} -> A vector of variable size. Check constraints for sizes. Constant.

# Constraints
- res should be a vector, not a matrix.
- vec should be a vector, not a matrix.
- res and vec should have the same size

"""
function mju_n2d(
    res::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        throw(ArgumentError("vec should be a vector, not a matrix."))
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return LibMuJoCo.mju_n2d(res, vec, length(res))

end
"""
    mju_insertionSort(res)

Insertion sort, resulting list is in increasing order.

# Arguments
- res::Vector{Float64} -> A vector of variable size. Check constraints for sizes.

# Constraints
- res should be a vector, not a matrix.

"""
function mju_insertionSort(res::Union{AbstractVector{Float64},AbstractArray{Float64,2}})
    if typeof(res) <: AbstractArray{Float64,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_insertionSort(res, length(res))

end
"""
    mju_insertionSortInt(res)

Integer insertion sort, resulting list is in increasing order.

# Arguments
- res::Vector{Int32} -> A vector of variable size. Check constraints for sizes.

# Constraints
- res should be a vector, not a matrix.

"""
function mju_insertionSortInt(res::Union{AbstractVector{Int32},AbstractArray{Int32,2}})
    if typeof(res) <: AbstractArray{Int32,2} && count(==(1), size(res)) < 1
        throw(ArgumentError("res should be a vector, not a matrix."))
    end

    return LibMuJoCo.mju_insertionSortInt(res, length(res))

end
"""
    mjd_transitionFD(m, d, eps, flg_centered, A, B, C, D)

Finite differenced transition matrices (control theory notation)   d(x_next) = A*dx + B*du   d(sensor) = C*dx + D*du   required output matrix dimensions:      A: (2*nv+na x 2*nv+na)      B: (2*nv+na x nu)      D: (nsensordata x 2*nv+na)      C: (nsensordata x nu)

# Arguments
- m::Model -> Constant.
- d::Data
- eps::Float64
- flg_centered::UInt8
- A::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- B::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- C::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- D::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.

# Constraints
- A should be of shape (2*nv+na, 2*nv+na)
- B should be of shape (2*nv+na, nu)
- C should be of shape (nsensordata, 2*nv+na)
- D should be of shape (nsensordata, nu)

"""
function mjd_transitionFD(
    m,
    d,
    eps::Number,
    flg_centered::UInt8,
    A::Union{Nothing,AbstractArray{Float64,2}},
    B::Union{Nothing,AbstractArray{Float64,2}},
    C::Union{Nothing,AbstractArray{Float64,2}},
    D::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(A) && !(typeof(A) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("A")
    end
    if !isnothing(B) && !(typeof(B) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("B")
    end
    if !isnothing(C) && !(typeof(C) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("C")
    end
    if !isnothing(D) && !(typeof(D) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("D")
    end

    if (!isnothing(A) && (size(A, 1) != 2 * m.nv + m.na || size(A, 2) != 2 * m.nv + m.na))
        throw(ArgumentError("A should be of shape (2*nv+na, 2*nv+na)"))
    end
    if (!isnothing(B) && (size(B, 1) != 2 * m.nv + m.na || size(B, 2) != m.nu))
        throw(ArgumentError("B should be of shape (2*nv+na, nu)"))
    end
    if (!isnothing(C) && (size(C, 1) != m.nsensordata || size(C, 2) != 2 * m.nv + m.na))
        throw(ArgumentError("C should be of shape (nsensordata, 2*nv+na)"))
    end
    if (!isnothing(D) && (size(D, 1) != m.nsensordata || size(D, 2) != m.nu))
        throw(ArgumentError("D should be of shape (nsensordata, nu)"))
    end
    return LibMuJoCo.mjd_transitionFD(
        m,
        d,
        eps,
        flg_centered,
        !isnothing(A) ? A : C_NULL,
        !isnothing(B) ? B : C_NULL,
        !isnothing(C) ? C : C_NULL,
        !isnothing(D) ? D : C_NULL,
    )

end
"""
    mjd_inverseFD(m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)

Finite differenced Jacobians of (force, sensors) = mj*inverse(state, acceleration)   All outputs are optional. Output dimensions (transposed w.r.t Control Theory convention):     DfDq: (nv x nv)     DfDv: (nv x nv)     DfDa: (nv x nv)     DsDq: (nv x nsensordata)     DsDv: (nv x nsensordata)     DsDa: (nv x nsensordata)     DmDq: (nv x nM)   single-letter shortcuts:     inputs: q=qpos, v=qvel, a=qacc     outputs: f=qfrc*inverse, s=sensordata, m=qM   notes:     optionally computes mass matrix Jacobian DmDq     flg*actuation specifies whether to subtract qfrc*actuator from qfrc_inverse

# Arguments
- m::Model -> Constant.
- d::Data
- eps::Float64
- flg_actuation::UInt8
- DfDq::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- DfDv::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- DfDa::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- DsDq::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- DsDv::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- DsDa::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.
- DmDq::Matrix{Float64} -> A matrix variable size. Check constraints for sizes.

# Constraints
- DfDq should be of shape (nv, nv)
- DfDv should be of shape (nv, nv)
- DfDa should be of shape (nv, nv)
- DsDq should be of shape (nv, nsensordata)
- DsDv should be of shape (nv, nsensordata)
- DsDa should be of shape (nv, nsensordata)
- DmDq should be of shape (nv, nM)

"""
function mjd_inverseFD(
    m,
    d,
    eps::Number,
    flg_actuation::UInt8,
    DfDq::Union{Nothing,AbstractArray{Float64,2}},
    DfDv::Union{Nothing,AbstractArray{Float64,2}},
    DfDa::Union{Nothing,AbstractArray{Float64,2}},
    DsDq::Union{Nothing,AbstractArray{Float64,2}},
    DsDv::Union{Nothing,AbstractArray{Float64,2}},
    DsDa::Union{Nothing,AbstractArray{Float64,2}},
    DmDq::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(DfDq) &&
       !(typeof(DfDq) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DfDq")
    end
    if !isnothing(DfDv) &&
       !(typeof(DfDv) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DfDv")
    end
    if !isnothing(DfDa) &&
       !(typeof(DfDa) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DfDa")
    end
    if !isnothing(DsDq) &&
       !(typeof(DsDq) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DsDq")
    end
    if !isnothing(DsDv) &&
       !(typeof(DsDv) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DsDv")
    end
    if !isnothing(DsDa) &&
       !(typeof(DsDa) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DsDa")
    end
    if !isnothing(DmDq) &&
       !(typeof(DmDq) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DmDq")
    end

    if (!isnothing(DfDq) && (size(DfDq, 1) != m.nv || size(DfDq, 2) != m.nv))
        throw(ArgumentError("DfDq should be of shape (nv, nv)"))
    end
    if (!isnothing(DfDv) && (size(DfDv, 1) != m.nv || size(DfDv, 2) != m.nv))
        throw(ArgumentError("DfDv should be of shape (nv, nv)"))
    end
    if (!isnothing(DfDa) && (size(DfDa, 1) != m.nv || size(DfDa, 2) != m.nv))
        throw(ArgumentError("DfDa should be of shape (nv, nv)"))
    end
    if (!isnothing(DsDq) && (size(DsDq, 1) != m.nv || size(DsDq, 2) != m.nsensordata))
        throw(ArgumentError("DsDq should be of shape (nv, nsensordata)"))
    end
    if (!isnothing(DsDv) && (size(DsDv, 1) != m.nv || size(DsDv, 2) != m.nsensordata))
        throw(ArgumentError("DsDv should be of shape (nv, nsensordata)"))
    end
    if (!isnothing(DsDa) && (size(DsDa, 1) != m.nv || size(DsDa, 2) != m.nsensordata))
        throw(ArgumentError("DsDa should be of shape (nv, nsensordata)"))
    end
    if (!isnothing(DmDq) && (size(DmDq, 1) != m.nv || size(DmDq, 2) != m.nM))
        throw(ArgumentError("DmDq should be of shape (nv, nM)"))
    end
    return LibMuJoCo.mjd_inverseFD(
        m,
        d,
        eps,
        flg_actuation,
        !isnothing(DfDq) ? DfDq : C_NULL,
        !isnothing(DfDv) ? DfDv : C_NULL,
        !isnothing(DfDa) ? DfDa : C_NULL,
        !isnothing(DsDq) ? DsDq : C_NULL,
        !isnothing(DsDv) ? DsDv : C_NULL,
        !isnothing(DsDa) ? DsDa : C_NULL,
        !isnothing(DmDq) ? DmDq : C_NULL,
    )

end

# Tuple for exports
const _wrapped_fns = (
    :mju_printMat,
    :mj_solveM,
    :mj_solveM2,
    :mj_rne,
    :mj_constraintUpdate,
    :mj_getState,
    :mj_setState,
    :mj_mulJacVec,
    :mj_mulJacTVec,
    :mj_jac,
    :mj_jacBody,
    :mj_jacBodyCom,
    :mj_jacSubtreeCom,
    :mj_jacGeom,
    :mj_jacSite,
    :mj_jacPointAxis,
    :mj_fullM,
    :mj_mulM,
    :mj_mulM2,
    :mj_addM,
    :mj_applyFT,
    :mj_differentiatePos,
    :mj_integratePos,
    :mj_normalizeQuat,
    :mj_ray,
    :mju_zero,
    :mju_fill,
    :mju_copy,
    :mju_sum,
    :mju_L1,
    :mju_scl,
    :mju_add,
    :mju_sub,
    :mju_addTo,
    :mju_subFrom,
    :mju_addToScl,
    :mju_addScl,
    :mju_normalize,
    :mju_norm,
    :mju_dot,
    :mju_mulMatVec,
    :mju_mulMatTVec,
    :mju_mulVecMatVec,
    :mju_transpose,
    :mju_symmetrize,
    :mju_eye,
    :mju_mulMatMat,
    :mju_mulMatMatT,
    :mju_mulMatTMat,
    :mju_sqrMatTD,
    :mju_cholFactor,
    :mju_cholSolve,
    :mju_cholUpdate,
    :mju_cholFactorBand,
    :mju_cholSolveBand,
    :mju_band2Dense,
    :mju_dense2Band,
    :mju_bandMulMatVec,
    :mju_boxQP,
    :mju_encodePyramid,
    :mju_decodePyramid,
    :mju_isZero,
    :mju_f2n,
    :mju_n2f,
    :mju_d2n,
    :mju_n2d,
    :mju_insertionSort,
    :mju_insertionSortInt,
    :mjd_transitionFD,
    :mjd_inverseFD,
)
