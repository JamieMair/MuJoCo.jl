module LibMuJoCo
using MuJoCo_jll
export MuJoCo_jll
using CEnum
include("consts.jl")
include("enums.jl")
include("structs.jl")
include("functions.jl")
const export_blacklist = (
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
    :mj_loadAllPluginLibraries,
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
for name in names(@__MODULE__(); all = true)
    if name in export_blacklist
        continue
    end
    name_str = string(name)
    if any((startswith(name_str, prefix) for prefix in PREFIXES))
        @eval export $name
    end
end
end
