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
function LibMuJoCo.mj_rne(m, d, flg_acc::Int32, result)

    if (length(result) != m.nv)
        throw(ArgumentError("result should have length nv"))
    end
    return mj_rne(m, d, flg_acc, result)

end
function LibMuJoCo.mj_constraintUpdate(
    m,
    d,
    jar,
    cost::Union{AbstractVector{Float64},AbstractArray{Float64,2}},
    flg_coneHessian::Int32,
)
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
function LibMuJoCo.mj_getState(m, d, state, spec::Int32)

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return mj_getState(m, d, state, spec)

end
function LibMuJoCo.mj_setState(m, d, state, spec::Int32)

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return mj_setState(m, d, state, spec)

end
function LibMuJoCo.mj_mulJacVec(m, d, res, vec)

    if (length(res) != d.nefc)
        throw(ArgumentError("res should be of length nefc"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of length nv"))
    end
    return mj_mulJacVec(m, d, res, vec)

end
function LibMuJoCo.mj_mulJacTVec(m, d, res, vec)

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
    point,
    body::Int32,
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
    body::Int32,
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
    body::Int32,
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
function LibMuJoCo.mj_jacSubtreeCom(m, d, jacp::AbstractArray{Float64,2}, body::Int32)
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
    geom::Int32,
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
    site::Int32,
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
    point,
    axis,
    body::Int32,
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
function LibMuJoCo.mj_fullM(m, dst::Union{Nothing,AbstractArray{Float64,2}}, M)
    if !isnothing(dst) && !(typeof(dst) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("dst")
    end

    if (length(M) != m.nM)
        throw(ArgumentError("M should be of size nM"))
    end
    if (size(dst, 2) != m.nv || size(dst, 1) != m.nv)
        throw(ArgumentError("dst should be of shape (nv, nv)"))
    end
    return mj_fullM(m, dst, M)

end
function LibMuJoCo.mj_mulM(m, d, res, vec)

    if (length(res) != m.nv)
        throw(ArgumentError("res should be of size nv"))
    end
    if (length(vec) != m.nv)
        throw(ArgumentError("vec should be of size nv"))
    end
    return mj_mulM(m, d, res, vec)

end
function LibMuJoCo.mj_mulM2(m, d, res, vec)

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
    dst,
    rownnz::Union{
        Nothing,
        AbstractVector{Int32},
        AbstractArray{Int32,2},
        NTuple{Eigen::Dynamic,Int32},
    },
    rowadr::Union{
        Nothing,
        AbstractVector{Int32},
        AbstractArray{Int32,2},
        NTuple{Eigen::Dynamic,Int32},
    },
    colind::Union{
        Nothing,
        AbstractVector{Int32},
        AbstractArray{Int32,2},
        NTuple{Eigen::Dynamic,Int32},
    },
)

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
function LibMuJoCo.mj_applyFT(m, d, force, torque, point, body::Int32, qfrc_target)

    if (length(qfrc_target) != m.nv)
        throw(ArgumentError("qfrc_target should be of size nv"))
    end
    return mj_applyFT(m, d, force[0+1], torque[0+1], point[0+1], body, qfrc_target)

end
function LibMuJoCo.mj_differentiatePos(m, qvel, dt::Float64, qpos1, qpos2)

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
function LibMuJoCo.mj_integratePos(m, qpos, qvel, dt::Float64)

    if (length(qpos) != m.nq)
        throw(ArgumentError("qpos should be of size nq"))
    end
    if (length(qvel) != m.nv)
        throw(ArgumentError("qvel should be of size nv"))
    end
    return mj_integratePos(m, qpos, qvel, dt)

end
function LibMuJoCo.mj_normalizeQuat(m, qpos)

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
    pnt,
    vec,
    geomgroup::Union{AbstractVector{UInt8},AbstractArray{UInt8,2}},
    flg_static::UInt8,
    bodyexclude::Int32,
    geomid,
)
    if length(geomgroup) != 6
        error("geomgroup should be a vector of size 6")
    end
    if typeof(geomgroup) <: AbstractArray{UInt8,2} && count(==(1), size(geomgroup)) < 1
        error("geomgroup should be a vector of size 6.")
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
function LibMuJoCo.mju_zero(res)

    return mju_zero(res, length(res))

end
function LibMuJoCo.mju_fill(res, val::Float64)

    return mju_fill(res, val, length(res))

end
function LibMuJoCo.mju_copy(res, data)

    if (length(res) != length(data))
        throw(ArgumentError("res and data should have the same size"))
    end
    return mju_copy(res, data, length(res))

end
function LibMuJoCo.mju_sum(vec)

    return mju_sum(vec, length(vec))

end
function LibMuJoCo.mju_L1(vec)

    return mju_L1(vec, length(vec))

end
function LibMuJoCo.mju_scl(res, vec, scl::Float64)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_scl(res, vec, scl, length(res))

end
function LibMuJoCo.mju_add(res, vec1, vec2)

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return mju_add(res, vec1, vec2, length(res))

end
function LibMuJoCo.mju_sub(res, vec1, vec2)

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return mju_sub(res, vec1, vec2, length(res))

end
function LibMuJoCo.mju_addTo(res, vec)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_addTo(res, vec, length(res))

end
function LibMuJoCo.mju_subFrom(res, vec)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_subFrom(res, vec, length(res))

end
function LibMuJoCo.mju_addToScl(res, vec, scl::Float64)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_addToScl(res, vec, scl, length(res))

end
function LibMuJoCo.mju_addScl(res, vec1, vec2, scl::Float64)

    if (length(res) != length(vec1))
        throw(ArgumentError("res and vec1 should have the same size"))
    end
    if (length(res) != length(vec2))
        throw(ArgumentError("res and vec2 should have the same size"))
    end
    return mju_addScl(res, vec1, vec2, scl, length(res))

end
function LibMuJoCo.mju_normalize(vec)

    return mju_normalize(vec, length(vec))

end
function LibMuJoCo.mju_norm(vec)

    return mju_norm(vec, length(vec))

end
function LibMuJoCo.mju_dot(vec1, vec2)

    if (length(vec1) != length(vec2))
        throw(ArgumentError("vec1 and vec2 should have the same size"))
    end
    return mju_dot(vec1, vec2, length(vec1))

end
function LibMuJoCo.mju_mulMatVec(res, mat::Union{Nothing,AbstractArray{Float64,2}}, vec)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (length(res) != size(mat, 1))
        throw(ArgumentError("size of res should equal the number of rows in mat"))
    end
    if (length(vec) != size(mat, 2))
        throw(ArgumentError("size of vec should equal the number of columns in mat"))
    end
    return mju_mulMatVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_mulMatTVec(res, mat::Union{Nothing,AbstractArray{Float64,2}}, vec)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
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
    vec1,
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec2,
)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
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
    diag,
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
    end
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
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
    mindiag::Float64,
)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
    end

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    return mju_cholFactor(mat, size(mat, 1), mindiag)

end
function LibMuJoCo.mju_cholSolve(res, mat::Union{Nothing,AbstractArray{Float64,2}}, vec)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
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
    x,
    flg_plus::Int32,
)
    if !isnothing(mat) && !(typeof(mat) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("mat")
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
    mat,
    ntotal::Int32,
    nband::Int32,
    ndense::Int32,
    diagadd::Float64,
    diagmul::Float64,
)

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(ArgumentError("mat must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    return mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)

end
function LibMuJoCo.mju_cholSolveBand(
    res,
    mat,
    vec,
    ntotal::Int32,
    nband::Int32,
    ndense::Int32,
)

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
    mat,
    ntotal::Int32,
    nband::Int32,
    ndense::Int32,
    flg_sym::UInt8,
)
    if !isnothing(res) && !(typeof(res) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("res")
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
    res,
    mat::Union{Nothing,AbstractArray{Float64,2}},
    ntotal::Int32,
    nband::Int32,
    ndense::Int32,
)
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
    res,
    mat::Union{Nothing,AbstractArray{Float64,2}},
    vec::Union{Nothing,AbstractArray{Float64,2}},
    ntotal::Int32,
    nband::Int32,
    ndense::Int32,
    nVec::Int32,
    flg_sym::UInt8,
)
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
    res,
    R::Union{Nothing,AbstractArray{Float64,2}},
    index::Union{AbstractVector{Int32},AbstractArray{Int32,2},NTuple{Eigen::Dynamic,Int32}},
    H::Union{Nothing,AbstractArray{Float64,2}},
    g,
    lower,
    upper,
)
    if !isnothing(R) && !(typeof(R) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("R")
    end
    if !isnothing(H) && !(typeof(H) <: LinearAlgebra.Transpose{Float64,Matrix{Float64}})
        @warn column_major_warning_string("H")
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
function LibMuJoCo.mju_encodePyramid(pyramid, force, mu)

    if (length(pyramid) != 2 * length(mu))
        throw(ArgumentError("size of pyramid should be twice as large as size of mu"))
    end
    if (length(force) != length(mu) + 1)
        throw(ArgumentError("size of force should be exactly one larger than size of mu"))
    end
    return mju_encodePyramid(pyramid, force, mu, length(mu))

end
function LibMuJoCo.mju_decodePyramid(force, pyramid, mu)

    if (length(pyramid) != 2 * length(mu))
        throw(ArgumentError("size of pyramid should be twice as large as size of mu"))
    end
    if (length(force) != length(mu) + 1)
        throw(ArgumentError("size of force should be exactly one larger than size of mu"))
    end
    return mju_decodePyramid(force, pyramid, mu, length(mu))

end
function LibMuJoCo.mju_isZero(vec)

    return mju_isZero(vec, length(vec))

end
function LibMuJoCo.mju_f2n(
    res,
    vec::Union{
        Nothing,
        AbstractVector{Float32},
        AbstractArray{Float32,2},
        NTuple{Eigen::Dynamic,Float32},
    },
)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_f2n(res, vec, length(res))

end
function LibMuJoCo.mju_n2f(
    res::Union{
        Nothing,
        AbstractVector{Float32},
        AbstractArray{Float32,2},
        NTuple{Eigen::Dynamic,Float32},
    },
    vec,
)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_n2f(res, vec, length(res))

end
function LibMuJoCo.mju_d2n(
    res,
    vec::Union{
        Nothing,
        AbstractVector{Float64},
        AbstractArray{Float64,2},
        NTuple{Eigen::Dynamic,Float64},
    },
)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_d2n(res, vec, length(res))

end
function LibMuJoCo.mju_n2d(
    res::Union{
        Nothing,
        AbstractVector{Float64},
        AbstractArray{Float64,2},
        NTuple{Eigen::Dynamic,Float64},
    },
    vec,
)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_n2d(res, vec, length(res))

end
function LibMuJoCo.mju_insertionSort(res)

    return mju_insertionSort(res, length(res))

end
function LibMuJoCo.mju_insertionSortInt(
    res::Union{
        Nothing,
        AbstractVector{Int32},
        AbstractArray{Int32,2},
        NTuple{Eigen::Dynamic,Int32},
    },
)

    return mju_insertionSortInt(res, length(res))

end
function LibMuJoCo.mjd_transitionFD(
    m,
    d,
    eps::Float64,
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
    eps::Float64,
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
