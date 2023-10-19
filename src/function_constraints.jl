import LinearAlgebra
function column_major_warning_string(variable_name)
    return "$variable_name is stored in column-major order (Julia default), but mujoco expects arrays in row-major order. Use helper functions to generate row-major arrays and see documentation for more details."
end

function LibMuJoCo.mju_printMat(mat::Union{Nothing,AbstractArray{Float64,2}})
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    return mju_printMat(mat, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mj_solveM(
    m,
    d,
    x::Union{Nothing,AbstractArray{Float64,2}},
    y::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(x) && !(typeof(x) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("x")
    end
    if !isnothing(y) && !(typeof(y) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mj_solveM(m, d, x, y, size(y, 1))

end
function LibMuJoCo.mj_solveM2(
    m,
    d,
    x::Union{Nothing,AbstractArray{Float64,2}},
    y::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(x) && !(typeof(x) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("x")
    end
    if !isnothing(y) && !(typeof(y) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mj_solveM2(m, d, x, y, size(y, 1))

end
function LibMuJoCo.mj_rne(
    m,
    d,
    flg_acc::Integer,
    result::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(result) &&
       typeof(result) <: AbstractArray{Float64,2} &&
       count(==(1), size(result)) < 1
        error("result should be a vector, not a matrix.")
    end

    if (length(result) != m.nv)
        throw(ArgumentError("result should have length nv"))
    end
    return mj_rne(m, d, flg_acc, result)

end
function LibMuJoCo.mj_constraintUpdate(
    m,
    d,
    jar::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    cost::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    flg_coneHessian::Integer,
)
    if !isnothing(jar) &&
       typeof(jar) <: AbstractArray{Float64,2} &&
       count(==(1), size(jar)) < 1
        error("jar should be a vector, not a matrix.")
    end
    if length(cost) != 1
        error("cost should be a vector of size 1")
    end
    if typeof(cost) <: AbstractArray{Float64,2} && count(==(1), size(cost)) < 1
        error("cost should be a vector of size 1.")
    end

    if (length(jar) != d.nefc)
        throw(ArgumentError("size of jar should equal nefc"))
    end
    return mj_constraintUpdate(m, d, jar, !isnothing(cost) ? cost : C_NULL, flg_coneHessian)

end
function LibMuJoCo.mj_getState(
    m,
    d,
    state::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    spec::Integer,
)
    if !isnothing(state) &&
       typeof(state) <: AbstractArray{Float64,2} &&
       count(==(1), size(state)) < 1
        error("state should be a vector, not a matrix.")
    end

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return mj_getState(m, d, state, spec)

end
function LibMuJoCo.mj_setState(
    m,
    d,
    state::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    spec::Integer,
)
    if !isnothing(state) &&
       typeof(state) <: AbstractArray{Float64,2} &&
       count(==(1), size(state)) < 1
        error("state should be a vector, not a matrix.")
    end

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return mj_setState(m, d, state, spec)

end
function LibMuJoCo.mj_mulJacVec(
    m,
    d,
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != d.nefc)
        throw(ArgumentError("res should be of length nefc"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of length nv"))
    end
    return mj_mulJacVec(m, d, res, vec)

end
function LibMuJoCo.mj_mulJacTVec(
    m,
    d,
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of length nv"))
    end
    if (length(vec) != d.nefc)
        throw(ArgumentError("vec should be of length nefc"))
    end
    return mj_mulJacTVec(m, d, res, vec)

end
function LibMuJoCo.mj_jac(
    m,
    d,
    jacp::AbstractArray{Float64,2},
    jacr::AbstractArray{Float64,2},
    point::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    body::Integer,
)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end
    if length(point) != 3
        error("point should be a vector of size 3")
    end
    if typeof(point) <: AbstractArray{Float64,2} && count(==(1), size(point)) < 1
        error("point should be a vector of size 3.")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return mj_jac(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        point[0+1],
        body,
    )

end
function LibMuJoCo.mj_jacBody(
    m,
    d,
    jacp::AbstractArray{Float64,2},
    jacr::AbstractArray{Float64,2},
    body::Integer,
)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return mj_jacBody(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        body,
    )

end
function LibMuJoCo.mj_jacBodyCom(
    m,
    d,
    jacp::AbstractArray{Float64,2},
    jacr::AbstractArray{Float64,2},
    body::Integer,
)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return mj_jacBodyCom(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        body,
    )

end
function LibMuJoCo.mj_jacSubtreeCom(m, d, jacp::AbstractArray{Float64,2}, body::Integer)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    return mj_jacSubtreeCom(m, d, !isnothing(jacp) ? jacp : C_NULL, body)

end
function LibMuJoCo.mj_jacGeom(
    m,
    d,
    jacp::AbstractArray{Float64,2},
    jacr::AbstractArray{Float64,2},
    geom::Integer,
)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return mj_jacGeom(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        geom,
    )

end
function LibMuJoCo.mj_jacSite(
    m,
    d,
    jacp::AbstractArray{Float64,2},
    jacr::AbstractArray{Float64,2},
    site::Integer,
)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return mj_jacSite(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        site,
    )

end
function LibMuJoCo.mj_jacPointAxis(
    m,
    d,
    jacp::AbstractArray{Float64,2},
    jacr::AbstractArray{Float64,2},
    point::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    axis::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    body::Integer,
)
    if !(typeof(jacp) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacp")
    end
    if !(typeof(jacr) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("jacr")
    end
    if length(point) != 3
        error("point should be a vector of size 3")
    end
    if typeof(point) <: AbstractArray{Float64,2} && count(==(1), size(point)) < 1
        error("point should be a vector of size 3.")
    end
    if length(axis) != 3
        error("axis should be a vector of size 3")
    end
    if typeof(axis) <: AbstractArray{Float64,2} && count(==(1), size(axis)) < 1
        error("axis should be a vector of size 3.")
    end

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    if (!isnothing(jacr) && (size(jacr, 1) != 3 || size(jacr, 2) != m.nv))
        throw(ArgumentError("jacr should be of shape (3, nv)"))
    end
    return mj_jacPointAxis(
        m,
        d,
        !isnothing(jacp) ? jacp : C_NULL,
        !isnothing(jacr) ? jacr : C_NULL,
        point[0+1],
        axis[0+1],
        body,
    )

end
function LibMuJoCo.mj_fullM(
    m,
    dst::Union{Nothing,AbstractArray{Float64,2}},
    M::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(dst) && !(typeof(dst) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("dst")
    end
    if !isnothing(M) && typeof(M) <: AbstractArray{Float64,2} && count(==(1), size(M)) < 1
        error("M should be a vector, not a matrix.")
    end

    if (length(M) != m.nM)
        throw(ArgumentError("M should be of size nM"))
    end
    if (size(dst, 2) != m.nv || size(dst, 1) != m.nv)
        throw(ArgumentError("dst should be of shape (nv, nv)"))
    end
    return mj_fullM(m, dst, M)

end
function LibMuJoCo.mj_mulM(
    m,
    d,
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of size nv"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of size nv"))
    end
    return mj_mulM(m, d, res, vec)

end
function LibMuJoCo.mj_mulM2(
    m,
    d,
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of size nv"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of size nv"))
    end
    return mj_mulM2(m, d, res, vec)

end
function LibMuJoCo.mj_addM(
    m,
    d,
    dst::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    rownnz::Union{Nothing,AbstractVector{Int32},AbstractArray{Int32,2}},
    rowadr::Union{Nothing,AbstractVector{Int32},AbstractArray{Int32,2}},
    colind::Union{Nothing,AbstractVector{Int32},AbstractArray{Int32,2}},
)
    if !isnothing(dst) &&
       typeof(dst) <: AbstractArray{Float64,2} &&
       count(==(1), size(dst)) < 1
        error("dst should be a vector, not a matrix.")
    end
    if !isnothing(rownnz) &&
       typeof(rownnz) <: AbstractArray{Int32,2} &&
       count(==(1), size(rownnz)) < 1
        error("rownnz should be a vector, not a matrix.")
    end
    if !isnothing(rowadr) &&
       typeof(rowadr) <: AbstractArray{Int32,2} &&
       count(==(1), size(rowadr)) < 1
        error("rowadr should be a vector, not a matrix.")
    end
    if !isnothing(colind) &&
       typeof(colind) <: AbstractArray{Int32,2} &&
       count(==(1), size(colind)) < 1
        error("colind should be a vector, not a matrix.")
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
    return mj_addM(m, d, dst, rownnz, rowadr, colind)

end
function LibMuJoCo.mj_applyFT(
    m,
    d,
    force::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    torque::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    point::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    body::Integer,
    qfrc_target::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if length(force) != 3
        error("force should be a vector of size 3")
    end
    if typeof(force) <: AbstractArray{Float64,2} && count(==(1), size(force)) < 1
        error("force should be a vector of size 3.")
    end
    if length(torque) != 3
        error("torque should be a vector of size 3")
    end
    if typeof(torque) <: AbstractArray{Float64,2} && count(==(1), size(torque)) < 1
        error("torque should be a vector of size 3.")
    end
    if length(point) != 3
        error("point should be a vector of size 3")
    end
    if typeof(point) <: AbstractArray{Float64,2} && count(==(1), size(point)) < 1
        error("point should be a vector of size 3.")
    end
    if !isnothing(qfrc_target) &&
       typeof(qfrc_target) <: AbstractArray{Float64,2} &&
       count(==(1), size(qfrc_target)) < 1
        error("qfrc_target should be a vector, not a matrix.")
    end

    if (length(qfrc_target) != m.nv)
        throw(ArgumentError("qfrc_target should be of size nv"))
    end
    return mj_applyFT(m, d, force[0+1], torque[0+1], point[0+1], body, qfrc_target)

end
function LibMuJoCo.mj_differentiatePos(
    m,
    qvel::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    dt::AbstractFloat,
    qpos1::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    qpos2::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(qvel) &&
       typeof(qvel) <: AbstractArray{Float64,2} &&
       count(==(1), size(qvel)) < 1
        error("qvel should be a vector, not a matrix.")
    end
    if !isnothing(qpos1) &&
       typeof(qpos1) <: AbstractArray{Float64,2} &&
       count(==(1), size(qpos1)) < 1
        error("qpos1 should be a vector, not a matrix.")
    end
    if !isnothing(qpos2) &&
       typeof(qpos2) <: AbstractArray{Float64,2} &&
       count(==(1), size(qpos2)) < 1
        error("qpos2 should be a vector, not a matrix.")
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
    return mj_differentiatePos(m, qvel, dt, qpos1, qpos2)

end
function LibMuJoCo.mj_integratePos(
    m,
    qpos::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    qvel::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    dt::AbstractFloat,
)
    if !isnothing(qpos) &&
       typeof(qpos) <: AbstractArray{Float64,2} &&
       count(==(1), size(qpos)) < 1
        error("qpos should be a vector, not a matrix.")
    end
    if !isnothing(qvel) &&
       typeof(qvel) <: AbstractArray{Float64,2} &&
       count(==(1), size(qvel)) < 1
        error("qvel should be a vector, not a matrix.")
    end

    if (length(qpos) != m.nq)
        throw(ArgumentError("qpos should be of size nq"))
    end
    if (length(qvel) != m.nv)
        throw(ArgumentError("qvel should be of size nv"))
    end
    return mj_integratePos(m, qpos, qvel, dt)

end
function LibMuJoCo.mj_normalizeQuat(
    m,
    qpos::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(qpos) &&
       typeof(qpos) <: AbstractArray{Float64,2} &&
       count(==(1), size(qpos)) < 1
        error("qpos should be a vector, not a matrix.")
    end

    if (length(qpos) != m.nq)
        throw(ArgumentError("qpos should be of size nq"))
    end
    return mj_normalizeQuat(m, qpos)

end
function LibMuJoCo.mj_loadAllPluginLibraries(directory::String)

    mj_loadAllPluginLibraries(directory, C_NULL)

end
function LibMuJoCo.mj_ray(
    m,
    d,
    pnt::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    geomgroup::Union{AbstractVector{UInt8},AbstractArray{UInt8,2}},
    flg_static::UInt8,
    bodyexclude::Integer,
    geomid::Union{AbstractVector{Int32},AbstractArray{Int32,2}},
)
    if length(pnt) != 3
        error("pnt should be a vector of size 3")
    end
    if typeof(pnt) <: AbstractArray{Float64,2} && count(==(1), size(pnt)) < 1
        error("pnt should be a vector of size 3.")
    end
    if length(vec) != 3
        error("vec should be a vector of size 3")
    end
    if typeof(vec) <: AbstractArray{Float64,2} && count(==(1), size(vec)) < 1
        error("vec should be a vector of size 3.")
    end
    if length(geomgroup) != 6
        error("geomgroup should be a vector of size 6")
    end
    if typeof(geomgroup) <: AbstractArray{UInt8,2} && count(==(1), size(geomgroup)) < 1
        error("geomgroup should be a vector of size 6.")
    end
    if length(geomid) != 1
        error("geomid should be a vector of size 1")
    end
    if typeof(geomid) <: AbstractArray{Int32,2} && count(==(1), size(geomid)) < 1
        error("geomid should be a vector of size 1.")
    end

    return mj_ray(
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
function LibMuJoCo.mju_zero(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end

    return mju_zero(res, length(res))

end
function LibMuJoCo.mju_fill(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    val::AbstractFloat,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end

    return mju_fill(res, val, length(res))

end
function LibMuJoCo.mju_copy(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    data::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(data) &&
       typeof(data) <: AbstractArray{Float64,2} &&
       count(==(1), size(data)) < 1
        error("data should be a vector, not a matrix.")
    end

    if (length(res) != length(data))
        throw(ArgumentError("res and data should have the same size"))
    end
    return mju_copy(res, data, length(res))

end
function LibMuJoCo.mju_sum(
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    return mju_sum(vec, length(vec))

end
function LibMuJoCo.mju_L1(
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    return mju_L1(vec, length(vec))

end
function LibMuJoCo.mju_scl(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    scl::AbstractFloat,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_scl(res, vec, scl, length(res))

end
function LibMuJoCo.mju_add(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec1::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec1) &&
       typeof(vec1) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec1)) < 1
        error("vec1 should be a vector, not a matrix.")
    end
    if !isnothing(vec2) &&
       typeof(vec2) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec2)) < 1
        error("vec2 should be a vector, not a matrix.")
    end

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return mju_add(res, vec1, vec2, length(res))

end
function LibMuJoCo.mju_sub(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec1::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec1) &&
       typeof(vec1) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec1)) < 1
        error("vec1 should be a vector, not a matrix.")
    end
    if !isnothing(vec2) &&
       typeof(vec2) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec2)) < 1
        error("vec2 should be a vector, not a matrix.")
    end

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return mju_sub(res, vec1, vec2, length(res))

end
function LibMuJoCo.mju_addTo(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_addTo(res, vec, length(res))

end
function LibMuJoCo.mju_subFrom(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_subFrom(res, vec, length(res))

end
function LibMuJoCo.mju_addToScl(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    scl::AbstractFloat,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_addToScl(res, vec, scl, length(res))

end
function LibMuJoCo.mju_addScl(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec1::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    scl::AbstractFloat,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec1) &&
       typeof(vec1) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec1)) < 1
        error("vec1 should be a vector, not a matrix.")
    end
    if !isnothing(vec2) &&
       typeof(vec2) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec2)) < 1
        error("vec2 should be a vector, not a matrix.")
    end

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return mju_addScl(res, vec1, vec2, scl, length(res))

end
function LibMuJoCo.mju_normalize(
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    return mju_normalize(vec, length(vec))

end
function LibMuJoCo.mju_norm(
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    return mju_norm(vec, length(vec))

end
function LibMuJoCo.mju_dot(
    vec1::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec2::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec1) &&
       typeof(vec1) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec1)) < 1
        error("vec1 should be a vector, not a matrix.")
    end
    if !isnothing(vec2) &&
       typeof(vec2) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec2)) < 1
        error("vec2 should be a vector, not a matrix.")
    end

    if (length(vec1) != length(vec2))
        throw(ArgumentError("vec1 and vec2 should have the same size"))
    end
    return mju_dot(vec1, vec2, length(vec1))

end
function LibMuJoCo.mju_mulMatVec(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != size(mat, 1))
        throw(ArgumentError("size of res should equal the number of rows in mat"))
    end
    if (length(vec) != size(mat, 2))
        throw(ArgumentError("size of vec should equal the number of columns in mat"))
    end
    return mju_mulMatVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_mulMatTVec(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != size(mat, 2))
        throw(ArgumentError("size of res should equal the number of columns in mat"))
    end
    if (length(vec) != size(mat, 1))
        throw(ArgumentError("size of vec should equal the number of rows in mat"))
    end
    return mju_mulMatTVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_mulVecMatVec(
    vec1::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec2::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec1) &&
       typeof(vec1) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec1)) < 1
        error("vec1 should be a vector, not a matrix.")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(vec2) &&
       typeof(vec2) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec2)) < 1
        error("vec2 should be a vector, not a matrix.")
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
    return mju_mulVecMatVec(vec1, mat, vec2, length(vec1))

end
function LibMuJoCo.mju_transpose(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(res, 2) != size(mat, 1))
        throw(ArgumentError("#columns in res should equal #rows in mat"))
    end
    if (size(res, 1) != size(mat, 2))
        throw(ArgumentError("#rows in res should equal #columns in mat"))
    end
    return mju_transpose(res, mat, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_symmetrize(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 2) != size(mat, 1))
        throw(ArgumentError("mat should be square"))
    end
    if (size(res, 2) != size(mat, 2) || size(res, 1) != size(mat, 1))
        throw(ArgumentError("res and mat should have the same shape"))
    end
    return mju_symmetrize(res, mat, size(mat, 1))

end
function LibMuJoCo.mju_eye(mat::Union{Nothing,AbstractArray{Float64,2}})
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 2) != size(mat, 1))
        throw(ArgumentError("mat should be square"))
    end
    return mju_eye(mat, size(mat, 1))

end
function LibMuJoCo.mju_mulMatMat(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat1::Union{Nothing,AbstractArray{Float64,2}},
    mat2::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat1) &&
       !(typeof(mat1) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat1")
    end
    if !isnothing(mat2) &&
       !(typeof(mat2) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mju_mulMatMat(res, mat1, mat2, size(mat1, 1), size(mat1, 2), size(mat2, 2))

end
function LibMuJoCo.mju_mulMatMatT(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat1::Union{Nothing,AbstractArray{Float64,2}},
    mat2::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat1) &&
       !(typeof(mat1) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat1")
    end
    if !isnothing(mat2) &&
       !(typeof(mat2) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mju_mulMatMatT(res, mat1, mat2, size(mat1, 1), size(mat1, 2), size(mat2, 1))

end
function LibMuJoCo.mju_mulMatTMat(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat1::Union{Nothing,AbstractArray{Float64,2}},
    mat2::Union{Nothing,AbstractArray{Float64,2}},
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat1) &&
       !(typeof(mat1) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat1")
    end
    if !isnothing(mat2) &&
       !(typeof(mat2) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mju_mulMatTMat(res, mat1, mat2, size(mat1, 1), size(mat1, 2), size(mat2, 2))

end
function LibMuJoCo.mju_sqrMatTD(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    diag::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if typeof(diag) <: AbstractArray{Float64,2} && count(==(1), size(diag)) < 1
        error("diag should be a vector, not a matrix.")
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
    return mju_sqrMatTD(
        res,
        mat,
        !isnothing(diag) ? diag : C_NULL,
        size(mat, 1),
        size(mat, 2),
    )

end
function LibMuJoCo.mju_cholFactor(
    mat::Union{Nothing,AbstractArray{Float64,2}},
    mindiag::AbstractFloat,
)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    return mju_cholFactor(mat, size(mat, 1), mindiag)

end
function LibMuJoCo.mju_cholSolve(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
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
    return mju_cholSolve(res, mat, vec, size(mat, 1))

end
function LibMuJoCo.mju_cholUpdate(
    mat::Union{Nothing,AbstractArray{Float64,2}},
    x::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    flg_plus::Integer,
)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(x) && typeof(x) <: AbstractArray{Float64,2} && count(==(1), size(x)) < 1
        error("x should be a vector, not a matrix.")
    end

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    if (length(x) != size(mat, 1))
        throw(ArgumentError("size of x should equal the number of rows in mat"))
    end
    return mju_cholUpdate(mat, x, size(mat, 1), flg_plus)

end
function LibMuJoCo.mju_cholFactorBand(
    mat::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
    diagadd::AbstractFloat,
    diagmul::AbstractFloat,
)
    if !isnothing(mat) &&
       typeof(mat) <: AbstractArray{Float64,2} &&
       count(==(1), size(mat)) < 1
        error("mat should be a vector, not a matrix.")
    end

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(ArgumentError("mat must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    return mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)

end
function LibMuJoCo.mju_cholSolveBand(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(mat) &&
       typeof(mat) <: AbstractArray{Float64,2} &&
       count(==(1), size(mat)) < 1
        error("mat should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
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
    return mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)

end
function LibMuJoCo.mju_band2Dense(
    res::Union{Nothing,AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
    flg_sym::UInt8,
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat) &&
       typeof(mat) <: AbstractArray{Float64,2} &&
       count(==(1), size(mat)) < 1
        error("mat should be a vector, not a matrix.")
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
    return mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)

end
function LibMuJoCo.mju_dense2Band(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mju_dense2Band(res, mat, ntotal, nband, ndense)

end
function LibMuJoCo.mju_bandMulMatVec(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractArray{Float64,2}},
    ntotal::Integer,
    nband::Integer,
    ndense::Integer,
    nVec::Integer,
    flg_sym::UInt8,
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end
    if !isnothing(vec) && !(typeof(vec) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nVec, flg_sym)

end
function LibMuJoCo.mju_boxQP(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    R::Union{Nothing,AbstractArray{Float64,2}},
    index::Union{AbstractVector{Int32},AbstractArray{Int32,2}},
    H::Union{Nothing,AbstractArray{Float64,2}},
    g::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    lower::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    upper::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(R) && !(typeof(R) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("R")
    end
    if typeof(index) <: AbstractArray{Int32,2} && count(==(1), size(index)) < 1
        error("index should be a vector, not a matrix.")
    end
    if !isnothing(H) && !(typeof(H) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("H")
    end
    if !isnothing(g) && typeof(g) <: AbstractArray{Float64,2} && count(==(1), size(g)) < 1
        error("g should be a vector, not a matrix.")
    end
    if typeof(lower) <: AbstractArray{Float64,2} && count(==(1), size(lower)) < 1
        error("lower should be a vector, not a matrix.")
    end
    if typeof(upper) <: AbstractArray{Float64,2} && count(==(1), size(upper)) < 1
        error("upper should be a vector, not a matrix.")
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
    return mju_boxQP(
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
function LibMuJoCo.mju_encodePyramid(
    pyramid::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    force::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mu::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(pyramid) &&
       typeof(pyramid) <: AbstractArray{Float64,2} &&
       count(==(1), size(pyramid)) < 1
        error("pyramid should be a vector, not a matrix.")
    end
    if !isnothing(force) &&
       typeof(force) <: AbstractArray{Float64,2} &&
       count(==(1), size(force)) < 1
        error("force should be a vector, not a matrix.")
    end
    if !isnothing(mu) &&
       typeof(mu) <: AbstractArray{Float64,2} &&
       count(==(1), size(mu)) < 1
        error("mu should be a vector, not a matrix.")
    end

    if (length(pyramid) != 2 * length(mu))
        throw(ArgumentError("size of pyramid should be twice as large as size of mu"))
    end
    if (length(force) != length(mu) + 1)
        throw(ArgumentError("size of force should be exactly one larger than size of mu"))
    end
    return mju_encodePyramid(pyramid, force, mu, length(mu))

end
function LibMuJoCo.mju_decodePyramid(
    force::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    pyramid::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    mu::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(force) &&
       typeof(force) <: AbstractArray{Float64,2} &&
       count(==(1), size(force)) < 1
        error("force should be a vector, not a matrix.")
    end
    if !isnothing(pyramid) &&
       typeof(pyramid) <: AbstractArray{Float64,2} &&
       count(==(1), size(pyramid)) < 1
        error("pyramid should be a vector, not a matrix.")
    end
    if !isnothing(mu) &&
       typeof(mu) <: AbstractArray{Float64,2} &&
       count(==(1), size(mu)) < 1
        error("mu should be a vector, not a matrix.")
    end

    if (length(pyramid) != 2 * length(mu))
        throw(ArgumentError("size of pyramid should be twice as large as size of mu"))
    end
    if (length(force) != length(mu) + 1)
        throw(ArgumentError("size of force should be exactly one larger than size of mu"))
    end
    return mju_decodePyramid(force, pyramid, mu, length(mu))

end
function LibMuJoCo.mju_isZero(
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    return mju_isZero(vec, length(vec))

end
function LibMuJoCo.mju_f2n(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float32},AbstractArray{Float32,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float32,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_f2n(res, vec, length(res))

end
function LibMuJoCo.mju_n2f(
    res::Union{Nothing,AbstractVector{Float32},AbstractArray{Float32,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float32,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_n2f(res, vec, length(res))

end
function LibMuJoCo.mju_d2n(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_d2n(res, vec, length(res))

end
function LibMuJoCo.mju_n2d(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end
    if !isnothing(vec) &&
       typeof(vec) <: AbstractArray{Float64,2} &&
       count(==(1), size(vec)) < 1
        error("vec should be a vector, not a matrix.")
    end

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_n2d(res, vec, length(res))

end
function LibMuJoCo.mju_insertionSort(
    res::Union{Nothing,AbstractVector{Float64},AbstractArray{Float64,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Float64,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end

    return mju_insertionSort(res, length(res))

end
function LibMuJoCo.mju_insertionSortInt(
    res::Union{Nothing,AbstractVector{Int32},AbstractArray{Int32,2}},
)
    if !isnothing(res) &&
       typeof(res) <: AbstractArray{Int32,2} &&
       count(==(1), size(res)) < 1
        error("res should be a vector, not a matrix.")
    end

    return mju_insertionSortInt(res, length(res))

end
function LibMuJoCo.mjd_transitionFD(
    m,
    d,
    eps::AbstractFloat,
    flg_centered::UInt8,
    A::AbstractArray{Float64,2},
    B::AbstractArray{Float64,2},
    C::AbstractArray{Float64,2},
    D::AbstractArray{Float64,2},
)
    if !(typeof(A) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("A")
    end
    if !(typeof(B) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("B")
    end
    if !(typeof(C) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("C")
    end
    if !(typeof(D) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mjd_transitionFD(
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
function LibMuJoCo.mjd_inverseFD(
    m,
    d,
    eps::AbstractFloat,
    flg_actuation::UInt8,
    DfDq::AbstractArray{Float64,2},
    DfDv::AbstractArray{Float64,2},
    DfDa::AbstractArray{Float64,2},
    DsDq::AbstractArray{Float64,2},
    DsDv::AbstractArray{Float64,2},
    DsDa::AbstractArray{Float64,2},
    DmDq::AbstractArray{Float64,2},
)
    if !(typeof(DfDq) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DfDq")
    end
    if !(typeof(DfDv) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DfDv")
    end
    if !(typeof(DfDa) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DfDa")
    end
    if !(typeof(DsDq) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DsDq")
    end
    if !(typeof(DsDv) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DsDv")
    end
    if !(typeof(DsDa) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("DsDa")
    end
    if !(typeof(DmDq) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
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
    return mjd_inverseFD(
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
