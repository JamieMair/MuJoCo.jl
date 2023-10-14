function LibMuJoCo.mju_printMat(mat)

    return mju_printMat(mat, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mj_solveM(m, d, x, y)

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
function LibMuJoCo.mj_solveM2(m, d, x, y)

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
function LibMuJoCo.mj_rne(m, d, flg_acc, result)

    if (length(result) != m.nv)
        throw(ArgumentError("result should have length nv"))
    end
    return mj_rne(m, d, flg_acc, result)

end
function LibMuJoCo.mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)

    if (length(jar) != d.nefc)
        throw(ArgumentError("size of jar should equal nefc"))
    end
    return mj_constraintUpdate(m, d, jar, !isnothing(cost) ? cost : C_NULL, flg_coneHessian)

end
function LibMuJoCo.mj_getState(m, d, state, spec)

    if (length(state) != mj_stateSize(m, spec))
        throw(ArgumentError("state size should equal mj_stateSize(m, spec)"))
    end
    return mj_getState(m, d, state, spec)

end
function LibMuJoCo.mj_setState(m, d, state, spec)

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
function LibMuJoCo.mj_jac(m, d, jacp, jacr, point, body)

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
function LibMuJoCo.mj_jacBody(m, d, jacp, jacr, body)

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
function LibMuJoCo.mj_jacBodyCom(m, d, jacp, jacr, body)

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
function LibMuJoCo.mj_jacSubtreeCom(m, d, jacp, body)

    if (!isnothing(jacp) && (size(jacp, 1) != 3 || size(jacp, 2) != m.nv))
        throw(ArgumentError("jacp should be of shape (3, nv)"))
    end
    return mj_jacSubtreeCom(m, d, !isnothing(jacp) ? jacp : C_NULL, body)

end
function LibMuJoCo.mj_jacGeom(m, d, jacp, jacr, geom)

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
function LibMuJoCo.mj_jacSite(m, d, jacp, jacr, site)

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
function LibMuJoCo.mj_jacPointAxis(m, d, jacp, jacr, point, axis, body)

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
function LibMuJoCo.mj_fullM(m, dst, M)

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
function LibMuJoCo.mj_addM(m, d, dst, rownnz, rowadr, colind)

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
function LibMuJoCo.mj_applyFT(m, d, force, torque, point, body, qfrc_target)

    if (length(qfrc_target) != m.nv)
        throw(ArgumentError("qfrc_target should be of size nv"))
    end
    return mj_applyFT(m, d, force[0+1], torque[0+1], point[0+1], body, qfrc_target)

end
function LibMuJoCo.mj_differentiatePos(m, qvel, dt, qpos1, qpos2)

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
function LibMuJoCo.mj_integratePos(m, qpos, qvel, dt)

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
function LibMuJoCo.mj_loadAllPluginLibraries(directory)

    mj_loadAllPluginLibraries(directory, C_NULL)

end
function LibMuJoCo.mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)

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
function LibMuJoCo.mju_fill(res, val)

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
function LibMuJoCo.mju_scl(res, vec, scl)

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
function LibMuJoCo.mju_addToScl(res, vec, scl)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_addToScl(res, vec, scl, length(res))

end
function LibMuJoCo.mju_addScl(res, vec1, vec2, scl)

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
function LibMuJoCo.mju_mulMatVec(res, mat, vec)

    if (length(res) != size(mat, 1))
        throw(ArgumentError("size of res should equal the number of rows in mat"))
    end
    if (length(vec) != size(mat, 2))
        throw(ArgumentError("size of vec should equal the number of columns in mat"))
    end
    return mju_mulMatVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_mulMatTVec(res, mat, vec)

    if (length(res) != size(mat, 2))
        throw(ArgumentError("size of res should equal the number of columns in mat"))
    end
    if (length(vec) != size(mat, 1))
        throw(ArgumentError("size of vec should equal the number of rows in mat"))
    end
    return mju_mulMatTVec(res, mat, vec, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_mulVecMatVec(vec1, mat, vec2)

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
function LibMuJoCo.mju_transpose(res, mat)

    if (size(res, 2) != size(mat, 1))
        throw(ArgumentError("#columns in res should equal #rows in mat"))
    end
    if (size(res, 1) != size(mat, 2))
        throw(ArgumentError("#rows in res should equal #columns in mat"))
    end
    return mju_transpose(res, mat, size(mat, 1), size(mat, 2))

end
function LibMuJoCo.mju_symmetrize(res, mat)

    if (size(mat, 2) != size(mat, 1))
        throw(ArgumentError("mat should be square"))
    end
    if (size(res, 2) != size(mat, 2) || size(res, 1) != size(mat, 1))
        throw(ArgumentError("res and mat should have the same shape"))
    end
    return mju_symmetrize(res, mat, size(mat, 1))

end
function LibMuJoCo.mju_eye(mat)

    if (size(mat, 2) != size(mat, 1))
        throw(ArgumentError("mat should be square"))
    end
    return mju_eye(mat, size(mat, 1))

end
function LibMuJoCo.mju_mulMatMat(res, mat1, mat2)

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
function LibMuJoCo.mju_mulMatMatT(res, mat1, mat2)

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
function LibMuJoCo.mju_mulMatTMat(res, mat1, mat2)

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
function LibMuJoCo.mju_sqrMatTD(res, mat, diag)

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
function LibMuJoCo.mju_cholFactor(mat, mindiag)

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    return mju_cholFactor(mat, size(mat, 1), mindiag)

end
function LibMuJoCo.mju_cholSolve(res, mat, vec)

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
function LibMuJoCo.mju_cholUpdate(mat, x, flg_plus)

    if (size(mat, 1) != size(mat, 2))
        throw(ArgumentError("mat should be a square matrix"))
    end
    if (length(x) != size(mat, 1))
        throw(ArgumentError("size of x should equal the number of rows in mat"))
    end
    return mju_cholUpdate(mat, x, size(mat, 1), flg_plus)

end
function LibMuJoCo.mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)

    nMat = (ntotal - ndense) * nband + ndense * ntotal
    if (length(mat) != nMat)
        throw(ArgumentError("mat must have size (ntotal-ndense)*nband + ndense*ntotal"))
    end
    return mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)

end
function LibMuJoCo.mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)

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
function LibMuJoCo.mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)

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
function LibMuJoCo.mju_dense2Band(res, mat, ntotal, nband, ndense)

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
function LibMuJoCo.mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nVec, flg_sym)

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
function LibMuJoCo.mju_boxQP(res, R, index, H, g, lower, upper)

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
function LibMuJoCo.mju_f2n(res, vec)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_f2n(res, vec, length(res))

end
function LibMuJoCo.mju_n2f(res, vec)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_n2f(res, vec, length(res))

end
function LibMuJoCo.mju_d2n(res, vec)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_d2n(res, vec, length(res))

end
function LibMuJoCo.mju_n2d(res, vec)

    if (length(res) != length(vec))
        throw(ArgumentError("res and vec should have the same size"))
    end
    return mju_n2d(res, vec, length(res))

end
function LibMuJoCo.mju_insertionSort(res)

    return mju_insertionSort(res, length(res))

end
function LibMuJoCo.mju_insertionSortInt(res)

    return mju_insertionSortInt(res, length(res))

end
function LibMuJoCo.mjd_transitionFD(m, d, eps, flg_centered, A, B, C, D)

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
    eps,
    flg_actuation,
    DfDq,
    DfDv,
    DfDa,
    DsDq,
    DsDv,
    DsDa,
    DmDq,
)

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
