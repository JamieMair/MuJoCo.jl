module LibMuJoCo

using MuJoCo_jll
export MuJoCo_jll

using CEnum

@cenum mjtState_::UInt32 begin
    mjSTATE_TIME = 1
    mjSTATE_QPOS = 2
    mjSTATE_QVEL = 4
    mjSTATE_ACT = 8
    mjSTATE_WARMSTART = 16
    mjSTATE_CTRL = 32
    mjSTATE_QFRC_APPLIED = 64
    mjSTATE_XFRC_APPLIED = 128
    mjSTATE_MOCAP_POS = 256
    mjSTATE_MOCAP_QUAT = 512
    mjSTATE_USERDATA = 1024
    mjSTATE_PLUGIN = 2048
    mjNSTATE = 12
    mjSTATE_PHYSICS = 14
    mjSTATE_FULLPHYSICS = 2063
    mjSTATE_USER = 2016
    mjSTATE_INTEGRATION = 4095
end

const mjtState = mjtState_

@cenum mjtWarning_::UInt32 begin
    mjWARN_INERTIA = 0
    mjWARN_CONTACTFULL = 1
    mjWARN_CNSTRFULL = 2
    mjWARN_VGEOMFULL = 3
    mjWARN_BADQPOS = 4
    mjWARN_BADQVEL = 5
    mjWARN_BADQACC = 6
    mjWARN_BADCTRL = 7
    mjNWARNING = 8
end

const mjtWarning = mjtWarning_

@cenum mjtTimer_::UInt32 begin
    mjTIMER_STEP = 0
    mjTIMER_FORWARD = 1
    mjTIMER_INVERSE = 2
    mjTIMER_POSITION = 3
    mjTIMER_VELOCITY = 4
    mjTIMER_ACTUATION = 5
    mjTIMER_ACCELERATION = 6
    mjTIMER_CONSTRAINT = 7
    mjTIMER_POS_KINEMATICS = 8
    mjTIMER_POS_INERTIA = 9
    mjTIMER_POS_COLLISION = 10
    mjTIMER_POS_MAKE = 11
    mjTIMER_POS_PROJECT = 12
    mjNTIMER = 13
end

const mjtTimer = mjtTimer_

const mjtNum = Cdouble

struct mjContact_
    dist::mjtNum
    pos::NTuple{3, mjtNum}
    frame::NTuple{9, mjtNum}
    includemargin::mjtNum
    friction::NTuple{5, mjtNum}
    solref::NTuple{2, mjtNum}
    solreffriction::NTuple{2, mjtNum}
    solimp::NTuple{5, mjtNum}
    mu::mjtNum
    H::NTuple{36, mjtNum}
    dim::Cint
    geom1::Cint
    geom2::Cint
    exclude::Cint
    efc_address::Cint
end

const mjContact = mjContact_

struct mjWarningStat_
    lastinfo::Cint
    number::Cint
end

const mjWarningStat = mjWarningStat_

struct mjTimerStat_
    duration::mjtNum
    number::Cint
end

const mjTimerStat = mjTimerStat_

struct mjSolverStat_
    improvement::mjtNum
    gradient::mjtNum
    lineslope::mjtNum
    nactive::Cint
    nchange::Cint
    neval::Cint
    nupdate::Cint
end

const mjSolverStat = mjSolverStat_

const mjtByte = Cuchar

struct mjData_
    nstack::Cint
    nbuffer::Cint
    nplugin::Cint
    pstack::Csize_t
    parena::Csize_t
    maxuse_stack::Cint
    maxuse_arena::Csize_t
    maxuse_con::Cint
    maxuse_efc::Cint
    warning::NTuple{8, mjWarningStat}
    timer::NTuple{13, mjTimerStat}
    solver::NTuple{1000, mjSolverStat}
    solver_iter::Cint
    solver_nnz::Cint
    solver_fwdinv::NTuple{2, mjtNum}
    nbodypair_broad::Cint
    nbodypair_narrow::Cint
    ngeompair_mid::Cint
    ngeompair_narrow::Cint
    ne::Cint
    nf::Cint
    nefc::Cint
    nnzJ::Cint
    ncon::Cint
    time::mjtNum
    energy::NTuple{2, mjtNum}
    buffer::Ptr{Cvoid}
    arena::Ptr{Cvoid}
    qpos::Ptr{mjtNum}
    qvel::Ptr{mjtNum}
    act::Ptr{mjtNum}
    qacc_warmstart::Ptr{mjtNum}
    plugin_state::Ptr{mjtNum}
    ctrl::Ptr{mjtNum}
    qfrc_applied::Ptr{mjtNum}
    xfrc_applied::Ptr{mjtNum}
    mocap_pos::Ptr{mjtNum}
    mocap_quat::Ptr{mjtNum}
    qacc::Ptr{mjtNum}
    act_dot::Ptr{mjtNum}
    userdata::Ptr{mjtNum}
    sensordata::Ptr{mjtNum}
    plugin::Ptr{Cint}
    plugin_data::Ptr{Csize_t}
    xpos::Ptr{mjtNum}
    xquat::Ptr{mjtNum}
    xmat::Ptr{mjtNum}
    xipos::Ptr{mjtNum}
    ximat::Ptr{mjtNum}
    xanchor::Ptr{mjtNum}
    xaxis::Ptr{mjtNum}
    geom_xpos::Ptr{mjtNum}
    geom_xmat::Ptr{mjtNum}
    site_xpos::Ptr{mjtNum}
    site_xmat::Ptr{mjtNum}
    cam_xpos::Ptr{mjtNum}
    cam_xmat::Ptr{mjtNum}
    light_xpos::Ptr{mjtNum}
    light_xdir::Ptr{mjtNum}
    subtree_com::Ptr{mjtNum}
    cdof::Ptr{mjtNum}
    cinert::Ptr{mjtNum}
    ten_wrapadr::Ptr{Cint}
    ten_wrapnum::Ptr{Cint}
    ten_J_rownnz::Ptr{Cint}
    ten_J_rowadr::Ptr{Cint}
    ten_J_colind::Ptr{Cint}
    ten_length::Ptr{mjtNum}
    ten_J::Ptr{mjtNum}
    wrap_obj::Ptr{Cint}
    wrap_xpos::Ptr{mjtNum}
    actuator_length::Ptr{mjtNum}
    actuator_moment::Ptr{mjtNum}
    crb::Ptr{mjtNum}
    qM::Ptr{mjtNum}
    qLD::Ptr{mjtNum}
    qLDiagInv::Ptr{mjtNum}
    qLDiagSqrtInv::Ptr{mjtNum}
    bvh_active::Ptr{mjtByte}
    ten_velocity::Ptr{mjtNum}
    actuator_velocity::Ptr{mjtNum}
    cvel::Ptr{mjtNum}
    cdof_dot::Ptr{mjtNum}
    qfrc_bias::Ptr{mjtNum}
    qfrc_passive::Ptr{mjtNum}
    efc_vel::Ptr{mjtNum}
    efc_aref::Ptr{mjtNum}
    subtree_linvel::Ptr{mjtNum}
    subtree_angmom::Ptr{mjtNum}
    qH::Ptr{mjtNum}
    qHDiagInv::Ptr{mjtNum}
    D_rownnz::Ptr{Cint}
    D_rowadr::Ptr{Cint}
    D_colind::Ptr{Cint}
    B_rownnz::Ptr{Cint}
    B_rowadr::Ptr{Cint}
    B_colind::Ptr{Cint}
    qDeriv::Ptr{mjtNum}
    qLU::Ptr{mjtNum}
    actuator_force::Ptr{mjtNum}
    qfrc_actuator::Ptr{mjtNum}
    qfrc_smooth::Ptr{mjtNum}
    qacc_smooth::Ptr{mjtNum}
    qfrc_constraint::Ptr{mjtNum}
    qfrc_inverse::Ptr{mjtNum}
    cacc::Ptr{mjtNum}
    cfrc_int::Ptr{mjtNum}
    cfrc_ext::Ptr{mjtNum}
    contact::Ptr{mjContact}
    efc_type::Ptr{Cint}
    efc_id::Ptr{Cint}
    efc_J_rownnz::Ptr{Cint}
    efc_J_rowadr::Ptr{Cint}
    efc_J_rowsuper::Ptr{Cint}
    efc_J_colind::Ptr{Cint}
    efc_JT_rownnz::Ptr{Cint}
    efc_JT_rowadr::Ptr{Cint}
    efc_JT_rowsuper::Ptr{Cint}
    efc_JT_colind::Ptr{Cint}
    efc_J::Ptr{mjtNum}
    efc_JT::Ptr{mjtNum}
    efc_pos::Ptr{mjtNum}
    efc_margin::Ptr{mjtNum}
    efc_frictionloss::Ptr{mjtNum}
    efc_diagApprox::Ptr{mjtNum}
    efc_KBIP::Ptr{mjtNum}
    efc_D::Ptr{mjtNum}
    efc_R::Ptr{mjtNum}
    efc_b::Ptr{mjtNum}
    efc_force::Ptr{mjtNum}
    efc_state::Ptr{Cint}
    efc_AR_rownnz::Ptr{Cint}
    efc_AR_rowadr::Ptr{Cint}
    efc_AR_colind::Ptr{Cint}
    efc_AR::Ptr{mjtNum}
end

const mjData = mjData_

# typedef void ( * mjfGeneric ) ( const mjModel * m , mjData * d )
const mjfGeneric = Ptr{Cvoid}

# typedef int ( * mjfConFilt ) ( const mjModel * m , mjData * d , int geom1 , int geom2 )
const mjfConFilt = Ptr{Cvoid}

# typedef void ( * mjfSensor ) ( const mjModel * m , mjData * d , int stage )
const mjfSensor = Ptr{Cvoid}

# typedef mjtNum ( * mjfTime ) ( void )
const mjfTime = Ptr{Cvoid}

# typedef mjtNum ( * mjfAct ) ( const mjModel * m , const mjData * d , int id )
const mjfAct = Ptr{Cvoid}

# typedef int ( * mjfCollision ) ( const mjModel * m , const mjData * d , mjContact * con , int g1 , int g2 , mjtNum margin )
const mjfCollision = Ptr{Cvoid}

@cenum mjtDisableBit_::UInt32 begin
    mjDSBL_CONSTRAINT = 1
    mjDSBL_EQUALITY = 2
    mjDSBL_FRICTIONLOSS = 4
    mjDSBL_LIMIT = 8
    mjDSBL_CONTACT = 16
    mjDSBL_PASSIVE = 32
    mjDSBL_GRAVITY = 64
    mjDSBL_CLAMPCTRL = 128
    mjDSBL_WARMSTART = 256
    mjDSBL_FILTERPARENT = 512
    mjDSBL_ACTUATION = 1024
    mjDSBL_REFSAFE = 2048
    mjDSBL_SENSOR = 4096
    mjDSBL_MIDPHASE = 8192
    mjNDISABLE = 14
end

const mjtDisableBit = mjtDisableBit_

@cenum mjtEnableBit_::UInt32 begin
    mjENBL_OVERRIDE = 1
    mjENBL_ENERGY = 2
    mjENBL_FWDINV = 4
    mjENBL_SENSORNOISE = 8
    mjENBL_MULTICCD = 16
    mjNENABLE = 5
end

const mjtEnableBit = mjtEnableBit_

@cenum mjtJoint_::UInt32 begin
    mjJNT_FREE = 0
    mjJNT_BALL = 1
    mjJNT_SLIDE = 2
    mjJNT_HINGE = 3
end

const mjtJoint = mjtJoint_

@cenum mjtGeom_::UInt32 begin
    mjGEOM_PLANE = 0
    mjGEOM_HFIELD = 1
    mjGEOM_SPHERE = 2
    mjGEOM_CAPSULE = 3
    mjGEOM_ELLIPSOID = 4
    mjGEOM_CYLINDER = 5
    mjGEOM_BOX = 6
    mjGEOM_MESH = 7
    mjNGEOMTYPES = 8
    mjGEOM_ARROW = 100
    mjGEOM_ARROW1 = 101
    mjGEOM_ARROW2 = 102
    mjGEOM_LINE = 103
    mjGEOM_SKIN = 104
    mjGEOM_LABEL = 105
    mjGEOM_NONE = 1001
end

const mjtGeom = mjtGeom_

@cenum mjtCamLight_::UInt32 begin
    mjCAMLIGHT_FIXED = 0
    mjCAMLIGHT_TRACK = 1
    mjCAMLIGHT_TRACKCOM = 2
    mjCAMLIGHT_TARGETBODY = 3
    mjCAMLIGHT_TARGETBODYCOM = 4
end

const mjtCamLight = mjtCamLight_

@cenum mjtTexture_::UInt32 begin
    mjTEXTURE_2D = 0
    mjTEXTURE_CUBE = 1
    mjTEXTURE_SKYBOX = 2
end

const mjtTexture = mjtTexture_

@cenum mjtIntegrator_::UInt32 begin
    mjINT_EULER = 0
    mjINT_RK4 = 1
    mjINT_IMPLICIT = 2
    mjINT_IMPLICITFAST = 3
end

const mjtIntegrator = mjtIntegrator_

@cenum mjtCollision_::UInt32 begin
    mjCOL_ALL = 0
    mjCOL_PAIR = 1
    mjCOL_DYNAMIC = 2
end

const mjtCollision = mjtCollision_

@cenum mjtCone_::UInt32 begin
    mjCONE_PYRAMIDAL = 0
    mjCONE_ELLIPTIC = 1
end

const mjtCone = mjtCone_

@cenum mjtJacobian_::UInt32 begin
    mjJAC_DENSE = 0
    mjJAC_SPARSE = 1
    mjJAC_AUTO = 2
end

const mjtJacobian = mjtJacobian_

@cenum mjtSolver_::UInt32 begin
    mjSOL_PGS = 0
    mjSOL_CG = 1
    mjSOL_NEWTON = 2
end

const mjtSolver = mjtSolver_

@cenum mjtEq_::UInt32 begin
    mjEQ_CONNECT = 0
    mjEQ_WELD = 1
    mjEQ_JOINT = 2
    mjEQ_TENDON = 3
    mjEQ_DISTANCE = 4
end

const mjtEq = mjtEq_

@cenum mjtWrap_::UInt32 begin
    mjWRAP_NONE = 0
    mjWRAP_JOINT = 1
    mjWRAP_PULLEY = 2
    mjWRAP_SITE = 3
    mjWRAP_SPHERE = 4
    mjWRAP_CYLINDER = 5
end

const mjtWrap = mjtWrap_

@cenum mjtTrn_::UInt32 begin
    mjTRN_JOINT = 0
    mjTRN_JOINTINPARENT = 1
    mjTRN_SLIDERCRANK = 2
    mjTRN_TENDON = 3
    mjTRN_SITE = 4
    mjTRN_BODY = 5
    mjTRN_UNDEFINED = 1000
end

const mjtTrn = mjtTrn_

@cenum mjtDyn_::UInt32 begin
    mjDYN_NONE = 0
    mjDYN_INTEGRATOR = 1
    mjDYN_FILTER = 2
    mjDYN_MUSCLE = 3
    mjDYN_USER = 4
end

const mjtDyn = mjtDyn_

@cenum mjtGain_::UInt32 begin
    mjGAIN_FIXED = 0
    mjGAIN_AFFINE = 1
    mjGAIN_MUSCLE = 2
    mjGAIN_USER = 3
end

const mjtGain = mjtGain_

@cenum mjtBias_::UInt32 begin
    mjBIAS_NONE = 0
    mjBIAS_AFFINE = 1
    mjBIAS_MUSCLE = 2
    mjBIAS_USER = 3
end

const mjtBias = mjtBias_

@cenum mjtObj_::UInt32 begin
    mjOBJ_UNKNOWN = 0
    mjOBJ_BODY = 1
    mjOBJ_XBODY = 2
    mjOBJ_JOINT = 3
    mjOBJ_DOF = 4
    mjOBJ_GEOM = 5
    mjOBJ_SITE = 6
    mjOBJ_CAMERA = 7
    mjOBJ_LIGHT = 8
    mjOBJ_MESH = 9
    mjOBJ_SKIN = 10
    mjOBJ_HFIELD = 11
    mjOBJ_TEXTURE = 12
    mjOBJ_MATERIAL = 13
    mjOBJ_PAIR = 14
    mjOBJ_EXCLUDE = 15
    mjOBJ_EQUALITY = 16
    mjOBJ_TENDON = 17
    mjOBJ_ACTUATOR = 18
    mjOBJ_SENSOR = 19
    mjOBJ_NUMERIC = 20
    mjOBJ_TEXT = 21
    mjOBJ_TUPLE = 22
    mjOBJ_KEY = 23
    mjOBJ_PLUGIN = 24
end

const mjtObj = mjtObj_

@cenum mjtConstraint_::UInt32 begin
    mjCNSTR_EQUALITY = 0
    mjCNSTR_FRICTION_DOF = 1
    mjCNSTR_FRICTION_TENDON = 2
    mjCNSTR_LIMIT_JOINT = 3
    mjCNSTR_LIMIT_TENDON = 4
    mjCNSTR_CONTACT_FRICTIONLESS = 5
    mjCNSTR_CONTACT_PYRAMIDAL = 6
    mjCNSTR_CONTACT_ELLIPTIC = 7
end

const mjtConstraint = mjtConstraint_

@cenum mjtConstraintState_::UInt32 begin
    mjCNSTRSTATE_SATISFIED = 0
    mjCNSTRSTATE_QUADRATIC = 1
    mjCNSTRSTATE_LINEARNEG = 2
    mjCNSTRSTATE_LINEARPOS = 3
    mjCNSTRSTATE_CONE = 4
end

const mjtConstraintState = mjtConstraintState_

@cenum mjtSensor_::UInt32 begin
    mjSENS_TOUCH = 0
    mjSENS_ACCELEROMETER = 1
    mjSENS_VELOCIMETER = 2
    mjSENS_GYRO = 3
    mjSENS_FORCE = 4
    mjSENS_TORQUE = 5
    mjSENS_MAGNETOMETER = 6
    mjSENS_RANGEFINDER = 7
    mjSENS_JOINTPOS = 8
    mjSENS_JOINTVEL = 9
    mjSENS_TENDONPOS = 10
    mjSENS_TENDONVEL = 11
    mjSENS_ACTUATORPOS = 12
    mjSENS_ACTUATORVEL = 13
    mjSENS_ACTUATORFRC = 14
    mjSENS_JOINTACTFRC = 15
    mjSENS_BALLQUAT = 16
    mjSENS_BALLANGVEL = 17
    mjSENS_JOINTLIMITPOS = 18
    mjSENS_JOINTLIMITVEL = 19
    mjSENS_JOINTLIMITFRC = 20
    mjSENS_TENDONLIMITPOS = 21
    mjSENS_TENDONLIMITVEL = 22
    mjSENS_TENDONLIMITFRC = 23
    mjSENS_FRAMEPOS = 24
    mjSENS_FRAMEQUAT = 25
    mjSENS_FRAMEXAXIS = 26
    mjSENS_FRAMEYAXIS = 27
    mjSENS_FRAMEZAXIS = 28
    mjSENS_FRAMELINVEL = 29
    mjSENS_FRAMEANGVEL = 30
    mjSENS_FRAMELINACC = 31
    mjSENS_FRAMEANGACC = 32
    mjSENS_SUBTREECOM = 33
    mjSENS_SUBTREELINVEL = 34
    mjSENS_SUBTREEANGMOM = 35
    mjSENS_CLOCK = 36
    mjSENS_PLUGIN = 37
    mjSENS_USER = 38
end

const mjtSensor = mjtSensor_

@cenum mjtStage_::UInt32 begin
    mjSTAGE_NONE = 0
    mjSTAGE_POS = 1
    mjSTAGE_VEL = 2
    mjSTAGE_ACC = 3
end

const mjtStage = mjtStage_

@cenum mjtDataType_::UInt32 begin
    mjDATATYPE_REAL = 0
    mjDATATYPE_POSITIVE = 1
    mjDATATYPE_AXIS = 2
    mjDATATYPE_QUATERNION = 3
end

const mjtDataType = mjtDataType_

@cenum mjtLRMode_::UInt32 begin
    mjLRMODE_NONE = 0
    mjLRMODE_MUSCLE = 1
    mjLRMODE_MUSCLEUSER = 2
    mjLRMODE_ALL = 3
end

const mjtLRMode = mjtLRMode_

struct mjLROpt_
    mode::Cint
    useexisting::Cint
    uselimit::Cint
    accel::mjtNum
    maxforce::mjtNum
    timeconst::mjtNum
    timestep::mjtNum
    inttotal::mjtNum
    interval::mjtNum
    tolrange::mjtNum
end

const mjLROpt = mjLROpt_

struct mjVFS_
    nfile::Cint
    filename::NTuple{2000, NTuple{1000, Cchar}}
    filesize::NTuple{2000, Cint}
    filedata::NTuple{2000, Ptr{Cvoid}}
end

const mjVFS = mjVFS_

struct mjResource_
    name::Ptr{Cchar}
    data::Ptr{Cvoid}
    provider_data::Ptr{Cvoid}
    read::Ptr{Cvoid}
    close::Ptr{Cvoid}
    getdir::Ptr{Cvoid}
end

const mjResource = mjResource_

struct mjOption_
    timestep::mjtNum
    apirate::mjtNum
    impratio::mjtNum
    tolerance::mjtNum
    noslip_tolerance::mjtNum
    mpr_tolerance::mjtNum
    gravity::NTuple{3, mjtNum}
    wind::NTuple{3, mjtNum}
    magnetic::NTuple{3, mjtNum}
    density::mjtNum
    viscosity::mjtNum
    o_margin::mjtNum
    o_solref::NTuple{2, mjtNum}
    o_solimp::NTuple{5, mjtNum}
    integrator::Cint
    collision::Cint
    cone::Cint
    jacobian::Cint
    solver::Cint
    iterations::Cint
    noslip_iterations::Cint
    mpr_iterations::Cint
    disableflags::Cint
    enableflags::Cint
end

const mjOption = mjOption_

struct var"##Ctag#584"
    fovy::Cfloat
    ipd::Cfloat
    azimuth::Cfloat
    elevation::Cfloat
    linewidth::Cfloat
    glow::Cfloat
    realtime::Cfloat
    offwidth::Cint
    offheight::Cint
    ellipsoidinertia::Cint
end
function Base.getproperty(x::Ptr{var"##Ctag#584"}, f::Symbol)
    f === :fovy && return Ptr{Cfloat}(x + 0)
    f === :ipd && return Ptr{Cfloat}(x + 4)
    f === :azimuth && return Ptr{Cfloat}(x + 8)
    f === :elevation && return Ptr{Cfloat}(x + 12)
    f === :linewidth && return Ptr{Cfloat}(x + 16)
    f === :glow && return Ptr{Cfloat}(x + 20)
    f === :realtime && return Ptr{Cfloat}(x + 24)
    f === :offwidth && return Ptr{Cint}(x + 28)
    f === :offheight && return Ptr{Cint}(x + 32)
    f === :ellipsoidinertia && return Ptr{Cint}(x + 36)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#584", f::Symbol)
    r = Ref{var"##Ctag#584"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#584"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#584"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct var"##Ctag#585"
    shadowsize::Cint
    offsamples::Cint
    numslices::Cint
    numstacks::Cint
    numquads::Cint
end
function Base.getproperty(x::Ptr{var"##Ctag#585"}, f::Symbol)
    f === :shadowsize && return Ptr{Cint}(x + 0)
    f === :offsamples && return Ptr{Cint}(x + 4)
    f === :numslices && return Ptr{Cint}(x + 8)
    f === :numstacks && return Ptr{Cint}(x + 12)
    f === :numquads && return Ptr{Cint}(x + 16)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#585", f::Symbol)
    r = Ref{var"##Ctag#585"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#585"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#585"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct var"##Ctag#586"
    ambient::NTuple{3, Cfloat}
    diffuse::NTuple{3, Cfloat}
    specular::NTuple{3, Cfloat}
    active::Cint
end
function Base.getproperty(x::Ptr{var"##Ctag#586"}, f::Symbol)
    f === :ambient && return Ptr{NTuple{3, Cfloat}}(x + 0)
    f === :diffuse && return Ptr{NTuple{3, Cfloat}}(x + 12)
    f === :specular && return Ptr{NTuple{3, Cfloat}}(x + 24)
    f === :active && return Ptr{Cint}(x + 36)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#586", f::Symbol)
    r = Ref{var"##Ctag#586"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#586"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#586"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct var"##Ctag#587"
    stiffness::Cfloat
    stiffnessrot::Cfloat
    force::Cfloat
    torque::Cfloat
    alpha::Cfloat
    fogstart::Cfloat
    fogend::Cfloat
    znear::Cfloat
    zfar::Cfloat
    haze::Cfloat
    shadowclip::Cfloat
    shadowscale::Cfloat
    actuatortendon::Cfloat
end
function Base.getproperty(x::Ptr{var"##Ctag#587"}, f::Symbol)
    f === :stiffness && return Ptr{Cfloat}(x + 0)
    f === :stiffnessrot && return Ptr{Cfloat}(x + 4)
    f === :force && return Ptr{Cfloat}(x + 8)
    f === :torque && return Ptr{Cfloat}(x + 12)
    f === :alpha && return Ptr{Cfloat}(x + 16)
    f === :fogstart && return Ptr{Cfloat}(x + 20)
    f === :fogend && return Ptr{Cfloat}(x + 24)
    f === :znear && return Ptr{Cfloat}(x + 28)
    f === :zfar && return Ptr{Cfloat}(x + 32)
    f === :haze && return Ptr{Cfloat}(x + 36)
    f === :shadowclip && return Ptr{Cfloat}(x + 40)
    f === :shadowscale && return Ptr{Cfloat}(x + 44)
    f === :actuatortendon && return Ptr{Cfloat}(x + 48)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#587", f::Symbol)
    r = Ref{var"##Ctag#587"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#587"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#587"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct var"##Ctag#588"
    forcewidth::Cfloat
    contactwidth::Cfloat
    contactheight::Cfloat
    connect::Cfloat
    com::Cfloat
    camera::Cfloat
    light::Cfloat
    selectpoint::Cfloat
    jointlength::Cfloat
    jointwidth::Cfloat
    actuatorlength::Cfloat
    actuatorwidth::Cfloat
    framelength::Cfloat
    framewidth::Cfloat
    constraint::Cfloat
    slidercrank::Cfloat
end
function Base.getproperty(x::Ptr{var"##Ctag#588"}, f::Symbol)
    f === :forcewidth && return Ptr{Cfloat}(x + 0)
    f === :contactwidth && return Ptr{Cfloat}(x + 4)
    f === :contactheight && return Ptr{Cfloat}(x + 8)
    f === :connect && return Ptr{Cfloat}(x + 12)
    f === :com && return Ptr{Cfloat}(x + 16)
    f === :camera && return Ptr{Cfloat}(x + 20)
    f === :light && return Ptr{Cfloat}(x + 24)
    f === :selectpoint && return Ptr{Cfloat}(x + 28)
    f === :jointlength && return Ptr{Cfloat}(x + 32)
    f === :jointwidth && return Ptr{Cfloat}(x + 36)
    f === :actuatorlength && return Ptr{Cfloat}(x + 40)
    f === :actuatorwidth && return Ptr{Cfloat}(x + 44)
    f === :framelength && return Ptr{Cfloat}(x + 48)
    f === :framewidth && return Ptr{Cfloat}(x + 52)
    f === :constraint && return Ptr{Cfloat}(x + 56)
    f === :slidercrank && return Ptr{Cfloat}(x + 60)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#588", f::Symbol)
    r = Ref{var"##Ctag#588"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#588"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#588"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct var"##Ctag#589"
    fog::NTuple{4, Cfloat}
    haze::NTuple{4, Cfloat}
    force::NTuple{4, Cfloat}
    inertia::NTuple{4, Cfloat}
    joint::NTuple{4, Cfloat}
    actuator::NTuple{4, Cfloat}
    actuatornegative::NTuple{4, Cfloat}
    actuatorpositive::NTuple{4, Cfloat}
    com::NTuple{4, Cfloat}
    camera::NTuple{4, Cfloat}
    light::NTuple{4, Cfloat}
    selectpoint::NTuple{4, Cfloat}
    connect::NTuple{4, Cfloat}
    contactpoint::NTuple{4, Cfloat}
    contactforce::NTuple{4, Cfloat}
    contactfriction::NTuple{4, Cfloat}
    contacttorque::NTuple{4, Cfloat}
    contactgap::NTuple{4, Cfloat}
    rangefinder::NTuple{4, Cfloat}
    constraint::NTuple{4, Cfloat}
    slidercrank::NTuple{4, Cfloat}
    crankbroken::NTuple{4, Cfloat}
end
function Base.getproperty(x::Ptr{var"##Ctag#589"}, f::Symbol)
    f === :fog && return Ptr{NTuple{4, Cfloat}}(x + 0)
    f === :haze && return Ptr{NTuple{4, Cfloat}}(x + 16)
    f === :force && return Ptr{NTuple{4, Cfloat}}(x + 32)
    f === :inertia && return Ptr{NTuple{4, Cfloat}}(x + 48)
    f === :joint && return Ptr{NTuple{4, Cfloat}}(x + 64)
    f === :actuator && return Ptr{NTuple{4, Cfloat}}(x + 80)
    f === :actuatornegative && return Ptr{NTuple{4, Cfloat}}(x + 96)
    f === :actuatorpositive && return Ptr{NTuple{4, Cfloat}}(x + 112)
    f === :com && return Ptr{NTuple{4, Cfloat}}(x + 128)
    f === :camera && return Ptr{NTuple{4, Cfloat}}(x + 144)
    f === :light && return Ptr{NTuple{4, Cfloat}}(x + 160)
    f === :selectpoint && return Ptr{NTuple{4, Cfloat}}(x + 176)
    f === :connect && return Ptr{NTuple{4, Cfloat}}(x + 192)
    f === :contactpoint && return Ptr{NTuple{4, Cfloat}}(x + 208)
    f === :contactforce && return Ptr{NTuple{4, Cfloat}}(x + 224)
    f === :contactfriction && return Ptr{NTuple{4, Cfloat}}(x + 240)
    f === :contacttorque && return Ptr{NTuple{4, Cfloat}}(x + 256)
    f === :contactgap && return Ptr{NTuple{4, Cfloat}}(x + 272)
    f === :rangefinder && return Ptr{NTuple{4, Cfloat}}(x + 288)
    f === :constraint && return Ptr{NTuple{4, Cfloat}}(x + 304)
    f === :slidercrank && return Ptr{NTuple{4, Cfloat}}(x + 320)
    f === :crankbroken && return Ptr{NTuple{4, Cfloat}}(x + 336)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#589", f::Symbol)
    r = Ref{var"##Ctag#589"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#589"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#589"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct mjVisual_
    data::NTuple{568, UInt8}
end

function Base.getproperty(x::Ptr{mjVisual_}, f::Symbol)
    f === :_global && return Ptr{var"##Ctag#584"}(x + 0)
    f === :quality && return Ptr{var"##Ctag#585"}(x + 40)
    f === :headlight && return Ptr{var"##Ctag#586"}(x + 60)
    f === :map && return Ptr{var"##Ctag#587"}(x + 100)
    f === :scale && return Ptr{var"##Ctag#588"}(x + 152)
    f === :rgba && return Ptr{var"##Ctag#589"}(x + 216)
    return getfield(x, f)
end

function Base.getproperty(x::mjVisual_, f::Symbol)
    r = Ref{mjVisual_}(x)
    ptr = Base.unsafe_convert(Ptr{mjVisual_}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{mjVisual_}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const mjVisual = mjVisual_

struct mjStatistic_
    meaninertia::mjtNum
    meanmass::mjtNum
    meansize::mjtNum
    extent::mjtNum
    center::NTuple{3, mjtNum}
end

const mjStatistic = mjStatistic_

struct mjModel_
    nq::Cint
    nv::Cint
    nu::Cint
    na::Cint
    nbody::Cint
    nbvh::Cint
    njnt::Cint
    ngeom::Cint
    nsite::Cint
    ncam::Cint
    nlight::Cint
    nmesh::Cint
    nmeshvert::Cint
    nmeshnormal::Cint
    nmeshtexcoord::Cint
    nmeshface::Cint
    nmeshgraph::Cint
    nskin::Cint
    nskinvert::Cint
    nskintexvert::Cint
    nskinface::Cint
    nskinbone::Cint
    nskinbonevert::Cint
    nhfield::Cint
    nhfielddata::Cint
    ntex::Cint
    ntexdata::Cint
    nmat::Cint
    npair::Cint
    nexclude::Cint
    neq::Cint
    ntendon::Cint
    nwrap::Cint
    nsensor::Cint
    nnumeric::Cint
    nnumericdata::Cint
    ntext::Cint
    ntextdata::Cint
    ntuple::Cint
    ntupledata::Cint
    nkey::Cint
    nmocap::Cint
    nplugin::Cint
    npluginattr::Cint
    nuser_body::Cint
    nuser_jnt::Cint
    nuser_geom::Cint
    nuser_site::Cint
    nuser_cam::Cint
    nuser_tendon::Cint
    nuser_actuator::Cint
    nuser_sensor::Cint
    nnames::Cint
    nnames_map::Cint
    nM::Cint
    nD::Cint
    nB::Cint
    nemax::Cint
    njmax::Cint
    nconmax::Cint
    nstack::Cint
    nuserdata::Cint
    nsensordata::Cint
    npluginstate::Cint
    nbuffer::Cint
    opt::mjOption
    vis::mjVisual
    stat::mjStatistic
    buffer::Ptr{Cvoid}
    qpos0::Ptr{mjtNum}
    qpos_spring::Ptr{mjtNum}
    body_parentid::Ptr{Cint}
    body_rootid::Ptr{Cint}
    body_weldid::Ptr{Cint}
    body_mocapid::Ptr{Cint}
    body_jntnum::Ptr{Cint}
    body_jntadr::Ptr{Cint}
    body_dofnum::Ptr{Cint}
    body_dofadr::Ptr{Cint}
    body_geomnum::Ptr{Cint}
    body_geomadr::Ptr{Cint}
    body_simple::Ptr{mjtByte}
    body_sameframe::Ptr{mjtByte}
    body_pos::Ptr{mjtNum}
    body_quat::Ptr{mjtNum}
    body_ipos::Ptr{mjtNum}
    body_iquat::Ptr{mjtNum}
    body_mass::Ptr{mjtNum}
    body_subtreemass::Ptr{mjtNum}
    body_inertia::Ptr{mjtNum}
    body_invweight0::Ptr{mjtNum}
    body_gravcomp::Ptr{mjtNum}
    body_user::Ptr{mjtNum}
    body_plugin::Ptr{Cint}
    body_bvhadr::Ptr{Cint}
    body_bvhnum::Ptr{Cint}
    bvh_depth::Ptr{Cint}
    bvh_child::Ptr{Cint}
    bvh_geomid::Ptr{Cint}
    bvh_aabb::Ptr{mjtNum}
    jnt_type::Ptr{Cint}
    jnt_qposadr::Ptr{Cint}
    jnt_dofadr::Ptr{Cint}
    jnt_bodyid::Ptr{Cint}
    jnt_group::Ptr{Cint}
    jnt_limited::Ptr{mjtByte}
    jnt_actfrclimited::Ptr{mjtByte}
    jnt_solref::Ptr{mjtNum}
    jnt_solimp::Ptr{mjtNum}
    jnt_pos::Ptr{mjtNum}
    jnt_axis::Ptr{mjtNum}
    jnt_stiffness::Ptr{mjtNum}
    jnt_range::Ptr{mjtNum}
    jnt_actfrcrange::Ptr{mjtNum}
    jnt_margin::Ptr{mjtNum}
    jnt_user::Ptr{mjtNum}
    dof_bodyid::Ptr{Cint}
    dof_jntid::Ptr{Cint}
    dof_parentid::Ptr{Cint}
    dof_Madr::Ptr{Cint}
    dof_simplenum::Ptr{Cint}
    dof_solref::Ptr{mjtNum}
    dof_solimp::Ptr{mjtNum}
    dof_frictionloss::Ptr{mjtNum}
    dof_armature::Ptr{mjtNum}
    dof_damping::Ptr{mjtNum}
    dof_invweight0::Ptr{mjtNum}
    dof_M0::Ptr{mjtNum}
    geom_type::Ptr{Cint}
    geom_contype::Ptr{Cint}
    geom_conaffinity::Ptr{Cint}
    geom_condim::Ptr{Cint}
    geom_bodyid::Ptr{Cint}
    geom_dataid::Ptr{Cint}
    geom_matid::Ptr{Cint}
    geom_group::Ptr{Cint}
    geom_priority::Ptr{Cint}
    geom_sameframe::Ptr{mjtByte}
    geom_solmix::Ptr{mjtNum}
    geom_solref::Ptr{mjtNum}
    geom_solimp::Ptr{mjtNum}
    geom_size::Ptr{mjtNum}
    geom_aabb::Ptr{mjtNum}
    geom_rbound::Ptr{mjtNum}
    geom_pos::Ptr{mjtNum}
    geom_quat::Ptr{mjtNum}
    geom_friction::Ptr{mjtNum}
    geom_margin::Ptr{mjtNum}
    geom_gap::Ptr{mjtNum}
    geom_fluid::Ptr{mjtNum}
    geom_user::Ptr{mjtNum}
    geom_rgba::Ptr{Cfloat}
    site_type::Ptr{Cint}
    site_bodyid::Ptr{Cint}
    site_matid::Ptr{Cint}
    site_group::Ptr{Cint}
    site_sameframe::Ptr{mjtByte}
    site_size::Ptr{mjtNum}
    site_pos::Ptr{mjtNum}
    site_quat::Ptr{mjtNum}
    site_user::Ptr{mjtNum}
    site_rgba::Ptr{Cfloat}
    cam_mode::Ptr{Cint}
    cam_bodyid::Ptr{Cint}
    cam_targetbodyid::Ptr{Cint}
    cam_pos::Ptr{mjtNum}
    cam_quat::Ptr{mjtNum}
    cam_poscom0::Ptr{mjtNum}
    cam_pos0::Ptr{mjtNum}
    cam_mat0::Ptr{mjtNum}
    cam_fovy::Ptr{mjtNum}
    cam_ipd::Ptr{mjtNum}
    cam_user::Ptr{mjtNum}
    light_mode::Ptr{Cint}
    light_bodyid::Ptr{Cint}
    light_targetbodyid::Ptr{Cint}
    light_directional::Ptr{mjtByte}
    light_castshadow::Ptr{mjtByte}
    light_active::Ptr{mjtByte}
    light_pos::Ptr{mjtNum}
    light_dir::Ptr{mjtNum}
    light_poscom0::Ptr{mjtNum}
    light_pos0::Ptr{mjtNum}
    light_dir0::Ptr{mjtNum}
    light_attenuation::Ptr{Cfloat}
    light_cutoff::Ptr{Cfloat}
    light_exponent::Ptr{Cfloat}
    light_ambient::Ptr{Cfloat}
    light_diffuse::Ptr{Cfloat}
    light_specular::Ptr{Cfloat}
    mesh_vertadr::Ptr{Cint}
    mesh_vertnum::Ptr{Cint}
    mesh_faceadr::Ptr{Cint}
    mesh_facenum::Ptr{Cint}
    mesh_bvhadr::Ptr{Cint}
    mesh_bvhnum::Ptr{Cint}
    mesh_normaladr::Ptr{Cint}
    mesh_normalnum::Ptr{Cint}
    mesh_texcoordadr::Ptr{Cint}
    mesh_texcoordnum::Ptr{Cint}
    mesh_graphadr::Ptr{Cint}
    mesh_vert::Ptr{Cfloat}
    mesh_normal::Ptr{Cfloat}
    mesh_texcoord::Ptr{Cfloat}
    mesh_face::Ptr{Cint}
    mesh_facenormal::Ptr{Cint}
    mesh_facetexcoord::Ptr{Cint}
    mesh_graph::Ptr{Cint}
    skin_matid::Ptr{Cint}
    skin_group::Ptr{Cint}
    skin_rgba::Ptr{Cfloat}
    skin_inflate::Ptr{Cfloat}
    skin_vertadr::Ptr{Cint}
    skin_vertnum::Ptr{Cint}
    skin_texcoordadr::Ptr{Cint}
    skin_faceadr::Ptr{Cint}
    skin_facenum::Ptr{Cint}
    skin_boneadr::Ptr{Cint}
    skin_bonenum::Ptr{Cint}
    skin_vert::Ptr{Cfloat}
    skin_texcoord::Ptr{Cfloat}
    skin_face::Ptr{Cint}
    skin_bonevertadr::Ptr{Cint}
    skin_bonevertnum::Ptr{Cint}
    skin_bonebindpos::Ptr{Cfloat}
    skin_bonebindquat::Ptr{Cfloat}
    skin_bonebodyid::Ptr{Cint}
    skin_bonevertid::Ptr{Cint}
    skin_bonevertweight::Ptr{Cfloat}
    hfield_size::Ptr{mjtNum}
    hfield_nrow::Ptr{Cint}
    hfield_ncol::Ptr{Cint}
    hfield_adr::Ptr{Cint}
    hfield_data::Ptr{Cfloat}
    tex_type::Ptr{Cint}
    tex_height::Ptr{Cint}
    tex_width::Ptr{Cint}
    tex_adr::Ptr{Cint}
    tex_rgb::Ptr{mjtByte}
    mat_texid::Ptr{Cint}
    mat_texuniform::Ptr{mjtByte}
    mat_texrepeat::Ptr{Cfloat}
    mat_emission::Ptr{Cfloat}
    mat_specular::Ptr{Cfloat}
    mat_shininess::Ptr{Cfloat}
    mat_reflectance::Ptr{Cfloat}
    mat_rgba::Ptr{Cfloat}
    pair_dim::Ptr{Cint}
    pair_geom1::Ptr{Cint}
    pair_geom2::Ptr{Cint}
    pair_signature::Ptr{Cint}
    pair_solref::Ptr{mjtNum}
    pair_solreffriction::Ptr{mjtNum}
    pair_solimp::Ptr{mjtNum}
    pair_margin::Ptr{mjtNum}
    pair_gap::Ptr{mjtNum}
    pair_friction::Ptr{mjtNum}
    exclude_signature::Ptr{Cint}
    eq_type::Ptr{Cint}
    eq_obj1id::Ptr{Cint}
    eq_obj2id::Ptr{Cint}
    eq_active::Ptr{mjtByte}
    eq_solref::Ptr{mjtNum}
    eq_solimp::Ptr{mjtNum}
    eq_data::Ptr{mjtNum}
    tendon_adr::Ptr{Cint}
    tendon_num::Ptr{Cint}
    tendon_matid::Ptr{Cint}
    tendon_group::Ptr{Cint}
    tendon_limited::Ptr{mjtByte}
    tendon_width::Ptr{mjtNum}
    tendon_solref_lim::Ptr{mjtNum}
    tendon_solimp_lim::Ptr{mjtNum}
    tendon_solref_fri::Ptr{mjtNum}
    tendon_solimp_fri::Ptr{mjtNum}
    tendon_range::Ptr{mjtNum}
    tendon_margin::Ptr{mjtNum}
    tendon_stiffness::Ptr{mjtNum}
    tendon_damping::Ptr{mjtNum}
    tendon_frictionloss::Ptr{mjtNum}
    tendon_lengthspring::Ptr{mjtNum}
    tendon_length0::Ptr{mjtNum}
    tendon_invweight0::Ptr{mjtNum}
    tendon_user::Ptr{mjtNum}
    tendon_rgba::Ptr{Cfloat}
    wrap_type::Ptr{Cint}
    wrap_objid::Ptr{Cint}
    wrap_prm::Ptr{mjtNum}
    actuator_trntype::Ptr{Cint}
    actuator_dyntype::Ptr{Cint}
    actuator_gaintype::Ptr{Cint}
    actuator_biastype::Ptr{Cint}
    actuator_trnid::Ptr{Cint}
    actuator_actadr::Ptr{Cint}
    actuator_actnum::Ptr{Cint}
    actuator_group::Ptr{Cint}
    actuator_ctrllimited::Ptr{mjtByte}
    actuator_forcelimited::Ptr{mjtByte}
    actuator_actlimited::Ptr{mjtByte}
    actuator_dynprm::Ptr{mjtNum}
    actuator_gainprm::Ptr{mjtNum}
    actuator_biasprm::Ptr{mjtNum}
    actuator_ctrlrange::Ptr{mjtNum}
    actuator_forcerange::Ptr{mjtNum}
    actuator_actrange::Ptr{mjtNum}
    actuator_gear::Ptr{mjtNum}
    actuator_cranklength::Ptr{mjtNum}
    actuator_acc0::Ptr{mjtNum}
    actuator_length0::Ptr{mjtNum}
    actuator_lengthrange::Ptr{mjtNum}
    actuator_user::Ptr{mjtNum}
    actuator_plugin::Ptr{Cint}
    sensor_type::Ptr{Cint}
    sensor_datatype::Ptr{Cint}
    sensor_needstage::Ptr{Cint}
    sensor_objtype::Ptr{Cint}
    sensor_objid::Ptr{Cint}
    sensor_reftype::Ptr{Cint}
    sensor_refid::Ptr{Cint}
    sensor_dim::Ptr{Cint}
    sensor_adr::Ptr{Cint}
    sensor_cutoff::Ptr{mjtNum}
    sensor_noise::Ptr{mjtNum}
    sensor_user::Ptr{mjtNum}
    sensor_plugin::Ptr{Cint}
    plugin::Ptr{Cint}
    plugin_stateadr::Ptr{Cint}
    plugin_statenum::Ptr{Cint}
    plugin_attr::Ptr{Cchar}
    plugin_attradr::Ptr{Cint}
    numeric_adr::Ptr{Cint}
    numeric_size::Ptr{Cint}
    numeric_data::Ptr{mjtNum}
    text_adr::Ptr{Cint}
    text_size::Ptr{Cint}
    text_data::Ptr{Cchar}
    tuple_adr::Ptr{Cint}
    tuple_size::Ptr{Cint}
    tuple_objtype::Ptr{Cint}
    tuple_objid::Ptr{Cint}
    tuple_objprm::Ptr{mjtNum}
    key_time::Ptr{mjtNum}
    key_qpos::Ptr{mjtNum}
    key_qvel::Ptr{mjtNum}
    key_act::Ptr{mjtNum}
    key_mpos::Ptr{mjtNum}
    key_mquat::Ptr{mjtNum}
    key_ctrl::Ptr{mjtNum}
    name_bodyadr::Ptr{Cint}
    name_jntadr::Ptr{Cint}
    name_geomadr::Ptr{Cint}
    name_siteadr::Ptr{Cint}
    name_camadr::Ptr{Cint}
    name_lightadr::Ptr{Cint}
    name_meshadr::Ptr{Cint}
    name_skinadr::Ptr{Cint}
    name_hfieldadr::Ptr{Cint}
    name_texadr::Ptr{Cint}
    name_matadr::Ptr{Cint}
    name_pairadr::Ptr{Cint}
    name_excludeadr::Ptr{Cint}
    name_eqadr::Ptr{Cint}
    name_tendonadr::Ptr{Cint}
    name_actuatoradr::Ptr{Cint}
    name_sensoradr::Ptr{Cint}
    name_numericadr::Ptr{Cint}
    name_textadr::Ptr{Cint}
    name_tupleadr::Ptr{Cint}
    name_keyadr::Ptr{Cint}
    name_pluginadr::Ptr{Cint}
    names::Ptr{Cchar}
    names_map::Ptr{Cint}
end

const mjModel = mjModel_

# typedef int ( * mjfOpenResource ) ( mjResource * resource )
const mjfOpenResource = Ptr{Cvoid}

# typedef int ( * mjfReadResource ) ( mjResource * resource , const void * * buffer )
const mjfReadResource = Ptr{Cvoid}

# typedef void ( * mjfCloseResource ) ( mjResource * resource )
const mjfCloseResource = Ptr{Cvoid}

# typedef void ( * mjfGetResourceDir ) ( mjResource * resource , const char * * dir , int * ndir )
const mjfGetResourceDir = Ptr{Cvoid}

struct mjpResourceProvider_
    prefix::Ptr{Cchar}
    open::mjfOpenResource
    read::mjfReadResource
    close::mjfCloseResource
    getdir::mjfGetResourceDir
    data::Ptr{Cvoid}
end

const mjpResourceProvider = mjpResourceProvider_

@cenum mjtPluginCapabilityBit_::UInt32 begin
    mjPLUGIN_ACTUATOR = 1
    mjPLUGIN_SENSOR = 2
    mjPLUGIN_PASSIVE = 4
end

const mjtPluginCapabilityBit = mjtPluginCapabilityBit_

struct mjpPlugin_
    name::Ptr{Cchar}
    nattribute::Cint
    attributes::Ptr{Ptr{Cchar}}
    capabilityflags::Cint
    needstage::Cint
    nstate::Ptr{Cvoid}
    nsensordata::Ptr{Cvoid}
    init::Ptr{Cvoid}
    destroy::Ptr{Cvoid}
    copy::Ptr{Cvoid}
    reset::Ptr{Cvoid}
    compute::Ptr{Cvoid}
    advance::Ptr{Cvoid}
    visualize::Ptr{Cvoid}
end

const mjpPlugin = mjpPlugin_

# typedef void ( * mjfPluginLibraryLoadCallback ) ( const char * filename , int first , int count )
const mjfPluginLibraryLoadCallback = Ptr{Cvoid}

@cenum mjtGridPos_::UInt32 begin
    mjGRID_TOPLEFT = 0
    mjGRID_TOPRIGHT = 1
    mjGRID_BOTTOMLEFT = 2
    mjGRID_BOTTOMRIGHT = 3
end

const mjtGridPos = mjtGridPos_

@cenum mjtFramebuffer_::UInt32 begin
    mjFB_WINDOW = 0
    mjFB_OFFSCREEN = 1
end

const mjtFramebuffer = mjtFramebuffer_

@cenum mjtFontScale_::UInt32 begin
    mjFONTSCALE_50 = 50
    mjFONTSCALE_100 = 100
    mjFONTSCALE_150 = 150
    mjFONTSCALE_200 = 200
    mjFONTSCALE_250 = 250
    mjFONTSCALE_300 = 300
end

const mjtFontScale = mjtFontScale_

@cenum mjtFont_::UInt32 begin
    mjFONT_NORMAL = 0
    mjFONT_SHADOW = 1
    mjFONT_BIG = 2
end

const mjtFont = mjtFont_

struct mjrRect_
    left::Cint
    bottom::Cint
    width::Cint
    height::Cint
end

const mjrRect = mjrRect_

struct mjrContext_
    lineWidth::Cfloat
    shadowClip::Cfloat
    shadowScale::Cfloat
    fogStart::Cfloat
    fogEnd::Cfloat
    fogRGBA::NTuple{4, Cfloat}
    shadowSize::Cint
    offWidth::Cint
    offHeight::Cint
    offSamples::Cint
    fontScale::Cint
    auxWidth::NTuple{10, Cint}
    auxHeight::NTuple{10, Cint}
    auxSamples::NTuple{10, Cint}
    offFBO::Cuint
    offFBO_r::Cuint
    offColor::Cuint
    offColor_r::Cuint
    offDepthStencil::Cuint
    offDepthStencil_r::Cuint
    shadowFBO::Cuint
    shadowTex::Cuint
    auxFBO::NTuple{10, Cuint}
    auxFBO_r::NTuple{10, Cuint}
    auxColor::NTuple{10, Cuint}
    auxColor_r::NTuple{10, Cuint}
    ntexture::Cint
    textureType::NTuple{100, Cint}
    texture::NTuple{100, Cuint}
    basePlane::Cuint
    baseMesh::Cuint
    baseHField::Cuint
    baseBuiltin::Cuint
    baseFontNormal::Cuint
    baseFontShadow::Cuint
    baseFontBig::Cuint
    rangePlane::Cint
    rangeMesh::Cint
    rangeHField::Cint
    rangeBuiltin::Cint
    rangeFont::Cint
    nskin::Cint
    skinvertVBO::Ptr{Cuint}
    skinnormalVBO::Ptr{Cuint}
    skintexcoordVBO::Ptr{Cuint}
    skinfaceVBO::Ptr{Cuint}
    charWidth::NTuple{127, Cint}
    charWidthBig::NTuple{127, Cint}
    charHeight::Cint
    charHeightBig::Cint
    glInitialized::Cint
    windowAvailable::Cint
    windowSamples::Cint
    windowStereo::Cint
    windowDoublebuffer::Cint
    currentBuffer::Cint
    readPixelFormat::Cint
end

const mjrContext = mjrContext_

@cenum mjtButton_::UInt32 begin
    mjBUTTON_NONE = 0
    mjBUTTON_LEFT = 1
    mjBUTTON_RIGHT = 2
    mjBUTTON_MIDDLE = 3
end

const mjtButton = mjtButton_

@cenum mjtEvent_::UInt32 begin
    mjEVENT_NONE = 0
    mjEVENT_MOVE = 1
    mjEVENT_PRESS = 2
    mjEVENT_RELEASE = 3
    mjEVENT_SCROLL = 4
    mjEVENT_KEY = 5
    mjEVENT_RESIZE = 6
    mjEVENT_REDRAW = 7
    mjEVENT_FILESDROP = 8
end

const mjtEvent = mjtEvent_

@cenum mjtItem_::Int32 begin
    mjITEM_END = -2
    mjITEM_SECTION = -1
    mjITEM_SEPARATOR = 0
    mjITEM_STATIC = 1
    mjITEM_BUTTON = 2
    mjITEM_CHECKINT = 3
    mjITEM_CHECKBYTE = 4
    mjITEM_RADIO = 5
    mjITEM_RADIOLINE = 6
    mjITEM_SELECT = 7
    mjITEM_SLIDERINT = 8
    mjITEM_SLIDERNUM = 9
    mjITEM_EDITINT = 10
    mjITEM_EDITNUM = 11
    mjITEM_EDITFLOAT = 12
    mjITEM_EDITTXT = 13
    mjNITEM = 14
end

const mjtItem = mjtItem_

# typedef int ( * mjfItemEnable ) ( int category , void * data )
const mjfItemEnable = Ptr{Cvoid}

struct mjuiState_
    nrect::Cint
    rect::NTuple{25, mjrRect}
    userdata::Ptr{Cvoid}
    type::Cint
    left::Cint
    right::Cint
    middle::Cint
    doubleclick::Cint
    button::Cint
    buttontime::Cdouble
    x::Cdouble
    y::Cdouble
    dx::Cdouble
    dy::Cdouble
    sx::Cdouble
    sy::Cdouble
    control::Cint
    shift::Cint
    alt::Cint
    key::Cint
    keytime::Cdouble
    mouserect::Cint
    dragrect::Cint
    dragbutton::Cint
    dropcount::Cint
    droppaths::Ptr{Ptr{Cchar}}
end

const mjuiState = mjuiState_

struct mjuiThemeSpacing_
    total::Cint
    scroll::Cint
    label::Cint
    section::Cint
    itemside::Cint
    itemmid::Cint
    itemver::Cint
    texthor::Cint
    textver::Cint
    linescroll::Cint
    samples::Cint
end

const mjuiThemeSpacing = mjuiThemeSpacing_

struct mjuiThemeColor_
    master::NTuple{3, Cfloat}
    thumb::NTuple{3, Cfloat}
    secttitle::NTuple{3, Cfloat}
    sectfont::NTuple{3, Cfloat}
    sectsymbol::NTuple{3, Cfloat}
    sectpane::NTuple{3, Cfloat}
    shortcut::NTuple{3, Cfloat}
    fontactive::NTuple{3, Cfloat}
    fontinactive::NTuple{3, Cfloat}
    decorinactive::NTuple{3, Cfloat}
    decorinactive2::NTuple{3, Cfloat}
    button::NTuple{3, Cfloat}
    check::NTuple{3, Cfloat}
    radio::NTuple{3, Cfloat}
    select::NTuple{3, Cfloat}
    select2::NTuple{3, Cfloat}
    slider::NTuple{3, Cfloat}
    slider2::NTuple{3, Cfloat}
    edit::NTuple{3, Cfloat}
    edit2::NTuple{3, Cfloat}
    cursor::NTuple{3, Cfloat}
end

const mjuiThemeColor = mjuiThemeColor_

struct mjuiItemSingle_
    modifier::Cint
    shortcut::Cint
end

struct mjuiItemMulti_
    nelem::Cint
    name::NTuple{35, NTuple{40, Cchar}}
end

struct mjuiItemSlider_
    range::NTuple{2, Cdouble}
    divisions::Cdouble
end

struct mjuiItemEdit_
    nelem::Cint
    range::NTuple{7, NTuple{2, Cdouble}}
end

struct mjuiItem_
    data::NTuple{1488, UInt8}
end

function Base.getproperty(x::Ptr{mjuiItem_}, f::Symbol)
    f === :type && return Ptr{Cint}(x + 0)
    f === :name && return Ptr{NTuple{40, Cchar}}(x + 4)
    f === :state && return Ptr{Cint}(x + 44)
    f === :pdata && return Ptr{Ptr{Cvoid}}(x + 48)
    f === :sectionid && return Ptr{Cint}(x + 56)
    f === :itemid && return Ptr{Cint}(x + 60)
    f === :single && return Ptr{mjuiItemSingle_}(x + 64)
    f === :multi && return Ptr{mjuiItemMulti_}(x + 64)
    f === :slider && return Ptr{mjuiItemSlider_}(x + 64)
    f === :edit && return Ptr{mjuiItemEdit_}(x + 64)
    f === :rect && return Ptr{mjrRect}(x + 1472)
    return getfield(x, f)
end

function Base.getproperty(x::mjuiItem_, f::Symbol)
    r = Ref{mjuiItem_}(x)
    ptr = Base.unsafe_convert(Ptr{mjuiItem_}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{mjuiItem_}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const mjuiItem = mjuiItem_

struct mjuiSection_
    name::NTuple{40, Cchar}
    state::Cint
    modifier::Cint
    shortcut::Cint
    nitem::Cint
    item::NTuple{100, mjuiItem}
    rtitle::mjrRect
    rcontent::mjrRect
end

const mjuiSection = mjuiSection_

struct mjUI_
    spacing::mjuiThemeSpacing
    color::mjuiThemeColor
    predicate::mjfItemEnable
    userdata::Ptr{Cvoid}
    rectid::Cint
    auxid::Cint
    radiocol::Cint
    width::Cint
    height::Cint
    maxheight::Cint
    scroll::Cint
    mousesect::Cint
    mouseitem::Cint
    mousehelp::Cint
    editsect::Cint
    edititem::Cint
    editcursor::Cint
    editscroll::Cint
    edittext::NTuple{300, Cchar}
    editchanged::Ptr{mjuiItem}
    nsect::Cint
    sect::NTuple{10, mjuiSection}
end

const mjUI = mjUI_

struct mjuiDef_
    type::Cint
    name::NTuple{40, Cchar}
    state::Cint
    pdata::Ptr{Cvoid}
    other::NTuple{300, Cchar}
end

const mjuiDef = mjuiDef_

@cenum mjtCatBit_::UInt32 begin
    mjCAT_STATIC = 1
    mjCAT_DYNAMIC = 2
    mjCAT_DECOR = 4
    mjCAT_ALL = 7
end

const mjtCatBit = mjtCatBit_

@cenum mjtMouse_::UInt32 begin
    mjMOUSE_NONE = 0
    mjMOUSE_ROTATE_V = 1
    mjMOUSE_ROTATE_H = 2
    mjMOUSE_MOVE_V = 3
    mjMOUSE_MOVE_H = 4
    mjMOUSE_ZOOM = 5
    mjMOUSE_SELECT = 6
end

const mjtMouse = mjtMouse_

@cenum mjtPertBit_::UInt32 begin
    mjPERT_TRANSLATE = 1
    mjPERT_ROTATE = 2
end

const mjtPertBit = mjtPertBit_

@cenum mjtCamera_::UInt32 begin
    mjCAMERA_FREE = 0
    mjCAMERA_TRACKING = 1
    mjCAMERA_FIXED = 2
    mjCAMERA_USER = 3
end

const mjtCamera = mjtCamera_

@cenum mjtLabel_::UInt32 begin
    mjLABEL_NONE = 0
    mjLABEL_BODY = 1
    mjLABEL_JOINT = 2
    mjLABEL_GEOM = 3
    mjLABEL_SITE = 4
    mjLABEL_CAMERA = 5
    mjLABEL_LIGHT = 6
    mjLABEL_TENDON = 7
    mjLABEL_ACTUATOR = 8
    mjLABEL_CONSTRAINT = 9
    mjLABEL_SKIN = 10
    mjLABEL_SELECTION = 11
    mjLABEL_SELPNT = 12
    mjLABEL_CONTACTPOINT = 13
    mjLABEL_CONTACTFORCE = 14
    mjNLABEL = 15
end

const mjtLabel = mjtLabel_

@cenum mjtFrame_::UInt32 begin
    mjFRAME_NONE = 0
    mjFRAME_BODY = 1
    mjFRAME_GEOM = 2
    mjFRAME_SITE = 3
    mjFRAME_CAMERA = 4
    mjFRAME_LIGHT = 5
    mjFRAME_CONTACT = 6
    mjFRAME_WORLD = 7
    mjNFRAME = 8
end

const mjtFrame = mjtFrame_

@cenum mjtVisFlag_::UInt32 begin
    mjVIS_CONVEXHULL = 0
    mjVIS_TEXTURE = 1
    mjVIS_JOINT = 2
    mjVIS_CAMERA = 3
    mjVIS_ACTUATOR = 4
    mjVIS_ACTIVATION = 5
    mjVIS_LIGHT = 6
    mjVIS_TENDON = 7
    mjVIS_RANGEFINDER = 8
    mjVIS_CONSTRAINT = 9
    mjVIS_INERTIA = 10
    mjVIS_SCLINERTIA = 11
    mjVIS_PERTFORCE = 12
    mjVIS_PERTOBJ = 13
    mjVIS_CONTACTPOINT = 14
    mjVIS_CONTACTFORCE = 15
    mjVIS_CONTACTSPLIT = 16
    mjVIS_TRANSPARENT = 17
    mjVIS_AUTOCONNECT = 18
    mjVIS_COM = 19
    mjVIS_SELECT = 20
    mjVIS_STATIC = 21
    mjVIS_SKIN = 22
    mjVIS_MIDPHASE = 23
    mjVIS_MESHBVH = 24
    mjNVISFLAG = 25
end

const mjtVisFlag = mjtVisFlag_

@cenum mjtRndFlag_::UInt32 begin
    mjRND_SHADOW = 0
    mjRND_WIREFRAME = 1
    mjRND_REFLECTION = 2
    mjRND_ADDITIVE = 3
    mjRND_SKYBOX = 4
    mjRND_FOG = 5
    mjRND_HAZE = 6
    mjRND_SEGMENT = 7
    mjRND_IDCOLOR = 8
    mjRND_CULL_FACE = 9
    mjNRNDFLAG = 10
end

const mjtRndFlag = mjtRndFlag_

@cenum mjtStereo_::UInt32 begin
    mjSTEREO_NONE = 0
    mjSTEREO_QUADBUFFERED = 1
    mjSTEREO_SIDEBYSIDE = 2
end

const mjtStereo = mjtStereo_

struct mjvPerturb_
    select::Cint
    skinselect::Cint
    active::Cint
    active2::Cint
    refpos::NTuple{3, mjtNum}
    refquat::NTuple{4, mjtNum}
    refselpos::NTuple{3, mjtNum}
    localpos::NTuple{3, mjtNum}
    localmass::mjtNum
    scale::mjtNum
end

const mjvPerturb = mjvPerturb_

struct mjvCamera_
    type::Cint
    fixedcamid::Cint
    trackbodyid::Cint
    lookat::NTuple{3, mjtNum}
    distance::mjtNum
    azimuth::mjtNum
    elevation::mjtNum
end

const mjvCamera = mjvCamera_

struct mjvGLCamera_
    pos::NTuple{3, Cfloat}
    forward::NTuple{3, Cfloat}
    up::NTuple{3, Cfloat}
    frustum_center::Cfloat
    frustum_bottom::Cfloat
    frustum_top::Cfloat
    frustum_near::Cfloat
    frustum_far::Cfloat
end

const mjvGLCamera = mjvGLCamera_

struct mjvGeom_
    type::Cint
    dataid::Cint
    objtype::Cint
    objid::Cint
    category::Cint
    texid::Cint
    texuniform::Cint
    texcoord::Cint
    segid::Cint
    texrepeat::NTuple{2, Cfloat}
    size::NTuple{3, Cfloat}
    pos::NTuple{3, Cfloat}
    mat::NTuple{9, Cfloat}
    rgba::NTuple{4, Cfloat}
    emission::Cfloat
    specular::Cfloat
    shininess::Cfloat
    reflectance::Cfloat
    label::NTuple{100, Cchar}
    camdist::Cfloat
    modelrbound::Cfloat
    transparent::mjtByte
end

const mjvGeom = mjvGeom_

struct mjvLight_
    pos::NTuple{3, Cfloat}
    dir::NTuple{3, Cfloat}
    attenuation::NTuple{3, Cfloat}
    cutoff::Cfloat
    exponent::Cfloat
    ambient::NTuple{3, Cfloat}
    diffuse::NTuple{3, Cfloat}
    specular::NTuple{3, Cfloat}
    headlight::mjtByte
    directional::mjtByte
    castshadow::mjtByte
end

const mjvLight = mjvLight_

struct mjvOption_
    label::Cint
    frame::Cint
    geomgroup::NTuple{6, mjtByte}
    sitegroup::NTuple{6, mjtByte}
    jointgroup::NTuple{6, mjtByte}
    tendongroup::NTuple{6, mjtByte}
    actuatorgroup::NTuple{6, mjtByte}
    skingroup::NTuple{6, mjtByte}
    flags::NTuple{25, mjtByte}
    bvh_depth::Cint
end

const mjvOption = mjvOption_

struct mjvScene_
    maxgeom::Cint
    ngeom::Cint
    geoms::Ptr{mjvGeom}
    geomorder::Ptr{Cint}
    nskin::Cint
    skinfacenum::Ptr{Cint}
    skinvertadr::Ptr{Cint}
    skinvertnum::Ptr{Cint}
    skinvert::Ptr{Cfloat}
    skinnormal::Ptr{Cfloat}
    nlight::Cint
    lights::NTuple{100, mjvLight}
    camera::NTuple{2, mjvGLCamera}
    enabletransform::mjtByte
    translate::NTuple{3, Cfloat}
    rotate::NTuple{4, Cfloat}
    scale::Cfloat
    stereo::Cint
    flags::NTuple{10, mjtByte}
    framewidth::Cint
    framergb::NTuple{3, Cfloat}
end

const mjvScene = mjvScene_

struct mjvFigure_
    flg_legend::Cint
    flg_ticklabel::NTuple{2, Cint}
    flg_extend::Cint
    flg_barplot::Cint
    flg_selection::Cint
    flg_symmetric::Cint
    linewidth::Cfloat
    gridwidth::Cfloat
    gridsize::NTuple{2, Cint}
    gridrgb::NTuple{3, Cfloat}
    figurergba::NTuple{4, Cfloat}
    panergba::NTuple{4, Cfloat}
    legendrgba::NTuple{4, Cfloat}
    textrgb::NTuple{3, Cfloat}
    linergb::NTuple{100, NTuple{3, Cfloat}}
    range::NTuple{2, NTuple{2, Cfloat}}
    xformat::NTuple{20, Cchar}
    yformat::NTuple{20, Cchar}
    minwidth::NTuple{20, Cchar}
    title::NTuple{1000, Cchar}
    xlabel::NTuple{100, Cchar}
    linename::NTuple{100, NTuple{100, Cchar}}
    legendoffset::Cint
    subplot::Cint
    highlight::NTuple{2, Cint}
    highlightid::Cint
    selection::Cfloat
    linepnt::NTuple{100, Cint}
    linedata::NTuple{100, NTuple{2000, Cfloat}}
    xaxispixel::NTuple{2, Cint}
    yaxispixel::NTuple{2, Cint}
    xaxisdata::NTuple{2, Cfloat}
    yaxisdata::NTuple{2, Cfloat}
end

const mjvFigure = mjvFigure_

struct var"##Ctag#590"
    nu::Cint
    na::Cint
    nbody::Cint
    nbvh::Cint
    njnt::Cint
    ngeom::Cint
    nsite::Cint
    ncam::Cint
    nlight::Cint
    nmesh::Cint
    nskin::Cint
    nskinvert::Cint
    nskinface::Cint
    nskinbone::Cint
    nskinbonevert::Cint
    nmat::Cint
    neq::Cint
    ntendon::Cint
    nwrap::Cint
    nsensor::Cint
    nnames::Cint
    nsensordata::Cint
    opt::mjOption
    vis::mjVisual
    stat::mjStatistic
    body_parentid::Ptr{Cint}
    body_rootid::Ptr{Cint}
    body_weldid::Ptr{Cint}
    body_mocapid::Ptr{Cint}
    body_jntnum::Ptr{Cint}
    body_jntadr::Ptr{Cint}
    body_geomnum::Ptr{Cint}
    body_geomadr::Ptr{Cint}
    body_iquat::Ptr{mjtNum}
    body_mass::Ptr{mjtNum}
    body_inertia::Ptr{mjtNum}
    body_bvhadr::Ptr{Cint}
    body_bvhnum::Ptr{Cint}
    bvh_depth::Ptr{Cint}
    bvh_child::Ptr{Cint}
    bvh_geomid::Ptr{Cint}
    bvh_aabb::Ptr{mjtNum}
    jnt_type::Ptr{Cint}
    jnt_bodyid::Ptr{Cint}
    jnt_group::Ptr{Cint}
    geom_type::Ptr{Cint}
    geom_bodyid::Ptr{Cint}
    geom_dataid::Ptr{Cint}
    geom_matid::Ptr{Cint}
    geom_group::Ptr{Cint}
    geom_size::Ptr{mjtNum}
    geom_aabb::Ptr{mjtNum}
    geom_rbound::Ptr{mjtNum}
    geom_rgba::Ptr{Cfloat}
    site_type::Ptr{Cint}
    site_bodyid::Ptr{Cint}
    site_matid::Ptr{Cint}
    site_group::Ptr{Cint}
    site_size::Ptr{mjtNum}
    site_rgba::Ptr{Cfloat}
    cam_fovy::Ptr{mjtNum}
    cam_ipd::Ptr{mjtNum}
    light_directional::Ptr{mjtByte}
    light_castshadow::Ptr{mjtByte}
    light_active::Ptr{mjtByte}
    light_attenuation::Ptr{Cfloat}
    light_cutoff::Ptr{Cfloat}
    light_exponent::Ptr{Cfloat}
    light_ambient::Ptr{Cfloat}
    light_diffuse::Ptr{Cfloat}
    light_specular::Ptr{Cfloat}
    mesh_texcoordadr::Ptr{Cint}
    mesh_graphadr::Ptr{Cint}
    skin_matid::Ptr{Cint}
    skin_group::Ptr{Cint}
    skin_rgba::Ptr{Cfloat}
    skin_inflate::Ptr{Cfloat}
    skin_vertadr::Ptr{Cint}
    skin_vertnum::Ptr{Cint}
    skin_texcoordadr::Ptr{Cint}
    skin_faceadr::Ptr{Cint}
    skin_facenum::Ptr{Cint}
    skin_boneadr::Ptr{Cint}
    skin_bonenum::Ptr{Cint}
    skin_vert::Ptr{Cfloat}
    skin_face::Ptr{Cint}
    skin_bonevertadr::Ptr{Cint}
    skin_bonevertnum::Ptr{Cint}
    skin_bonebindpos::Ptr{Cfloat}
    skin_bonebindquat::Ptr{Cfloat}
    skin_bonebodyid::Ptr{Cint}
    skin_bonevertid::Ptr{Cint}
    skin_bonevertweight::Ptr{Cfloat}
    mat_texid::Ptr{Cint}
    mat_texuniform::Ptr{mjtByte}
    mat_texrepeat::Ptr{Cfloat}
    mat_emission::Ptr{Cfloat}
    mat_specular::Ptr{Cfloat}
    mat_shininess::Ptr{Cfloat}
    mat_reflectance::Ptr{Cfloat}
    mat_rgba::Ptr{Cfloat}
    eq_type::Ptr{Cint}
    eq_obj1id::Ptr{Cint}
    eq_obj2id::Ptr{Cint}
    eq_active::Ptr{mjtByte}
    eq_data::Ptr{mjtNum}
    tendon_num::Ptr{Cint}
    tendon_matid::Ptr{Cint}
    tendon_group::Ptr{Cint}
    tendon_limited::Ptr{mjtByte}
    tendon_width::Ptr{mjtNum}
    tendon_range::Ptr{mjtNum}
    tendon_stiffness::Ptr{mjtNum}
    tendon_damping::Ptr{mjtNum}
    tendon_frictionloss::Ptr{mjtNum}
    tendon_lengthspring::Ptr{mjtNum}
    tendon_rgba::Ptr{Cfloat}
    actuator_trntype::Ptr{Cint}
    actuator_dyntype::Ptr{Cint}
    actuator_trnid::Ptr{Cint}
    actuator_actadr::Ptr{Cint}
    actuator_actnum::Ptr{Cint}
    actuator_group::Ptr{Cint}
    actuator_ctrllimited::Ptr{mjtByte}
    actuator_actlimited::Ptr{mjtByte}
    actuator_ctrlrange::Ptr{mjtNum}
    actuator_actrange::Ptr{mjtNum}
    actuator_cranklength::Ptr{mjtNum}
    sensor_type::Ptr{Cint}
    sensor_objid::Ptr{Cint}
    sensor_adr::Ptr{Cint}
    name_bodyadr::Ptr{Cint}
    name_jntadr::Ptr{Cint}
    name_geomadr::Ptr{Cint}
    name_siteadr::Ptr{Cint}
    name_camadr::Ptr{Cint}
    name_lightadr::Ptr{Cint}
    name_eqadr::Ptr{Cint}
    name_tendonadr::Ptr{Cint}
    name_actuatoradr::Ptr{Cint}
    names::Ptr{Cchar}
end
function Base.getproperty(x::Ptr{var"##Ctag#590"}, f::Symbol)
    f === :nu && return Ptr{Cint}(x + 0)
    f === :na && return Ptr{Cint}(x + 4)
    f === :nbody && return Ptr{Cint}(x + 8)
    f === :nbvh && return Ptr{Cint}(x + 12)
    f === :njnt && return Ptr{Cint}(x + 16)
    f === :ngeom && return Ptr{Cint}(x + 20)
    f === :nsite && return Ptr{Cint}(x + 24)
    f === :ncam && return Ptr{Cint}(x + 28)
    f === :nlight && return Ptr{Cint}(x + 32)
    f === :nmesh && return Ptr{Cint}(x + 36)
    f === :nskin && return Ptr{Cint}(x + 40)
    f === :nskinvert && return Ptr{Cint}(x + 44)
    f === :nskinface && return Ptr{Cint}(x + 48)
    f === :nskinbone && return Ptr{Cint}(x + 52)
    f === :nskinbonevert && return Ptr{Cint}(x + 56)
    f === :nmat && return Ptr{Cint}(x + 60)
    f === :neq && return Ptr{Cint}(x + 64)
    f === :ntendon && return Ptr{Cint}(x + 68)
    f === :nwrap && return Ptr{Cint}(x + 72)
    f === :nsensor && return Ptr{Cint}(x + 76)
    f === :nnames && return Ptr{Cint}(x + 80)
    f === :nsensordata && return Ptr{Cint}(x + 84)
    f === :opt && return Ptr{mjOption}(x + 88)
    f === :vis && return Ptr{mjVisual}(x + 328)
    f === :stat && return Ptr{mjStatistic}(x + 896)
    f === :body_parentid && return Ptr{Ptr{Cint}}(x + 952)
    f === :body_rootid && return Ptr{Ptr{Cint}}(x + 960)
    f === :body_weldid && return Ptr{Ptr{Cint}}(x + 968)
    f === :body_mocapid && return Ptr{Ptr{Cint}}(x + 976)
    f === :body_jntnum && return Ptr{Ptr{Cint}}(x + 984)
    f === :body_jntadr && return Ptr{Ptr{Cint}}(x + 992)
    f === :body_geomnum && return Ptr{Ptr{Cint}}(x + 1000)
    f === :body_geomadr && return Ptr{Ptr{Cint}}(x + 1008)
    f === :body_iquat && return Ptr{Ptr{mjtNum}}(x + 1016)
    f === :body_mass && return Ptr{Ptr{mjtNum}}(x + 1024)
    f === :body_inertia && return Ptr{Ptr{mjtNum}}(x + 1032)
    f === :body_bvhadr && return Ptr{Ptr{Cint}}(x + 1040)
    f === :body_bvhnum && return Ptr{Ptr{Cint}}(x + 1048)
    f === :bvh_depth && return Ptr{Ptr{Cint}}(x + 1056)
    f === :bvh_child && return Ptr{Ptr{Cint}}(x + 1064)
    f === :bvh_geomid && return Ptr{Ptr{Cint}}(x + 1072)
    f === :bvh_aabb && return Ptr{Ptr{mjtNum}}(x + 1080)
    f === :jnt_type && return Ptr{Ptr{Cint}}(x + 1088)
    f === :jnt_bodyid && return Ptr{Ptr{Cint}}(x + 1096)
    f === :jnt_group && return Ptr{Ptr{Cint}}(x + 1104)
    f === :geom_type && return Ptr{Ptr{Cint}}(x + 1112)
    f === :geom_bodyid && return Ptr{Ptr{Cint}}(x + 1120)
    f === :geom_dataid && return Ptr{Ptr{Cint}}(x + 1128)
    f === :geom_matid && return Ptr{Ptr{Cint}}(x + 1136)
    f === :geom_group && return Ptr{Ptr{Cint}}(x + 1144)
    f === :geom_size && return Ptr{Ptr{mjtNum}}(x + 1152)
    f === :geom_aabb && return Ptr{Ptr{mjtNum}}(x + 1160)
    f === :geom_rbound && return Ptr{Ptr{mjtNum}}(x + 1168)
    f === :geom_rgba && return Ptr{Ptr{Cfloat}}(x + 1176)
    f === :site_type && return Ptr{Ptr{Cint}}(x + 1184)
    f === :site_bodyid && return Ptr{Ptr{Cint}}(x + 1192)
    f === :site_matid && return Ptr{Ptr{Cint}}(x + 1200)
    f === :site_group && return Ptr{Ptr{Cint}}(x + 1208)
    f === :site_size && return Ptr{Ptr{mjtNum}}(x + 1216)
    f === :site_rgba && return Ptr{Ptr{Cfloat}}(x + 1224)
    f === :cam_fovy && return Ptr{Ptr{mjtNum}}(x + 1232)
    f === :cam_ipd && return Ptr{Ptr{mjtNum}}(x + 1240)
    f === :light_directional && return Ptr{Ptr{mjtByte}}(x + 1248)
    f === :light_castshadow && return Ptr{Ptr{mjtByte}}(x + 1256)
    f === :light_active && return Ptr{Ptr{mjtByte}}(x + 1264)
    f === :light_attenuation && return Ptr{Ptr{Cfloat}}(x + 1272)
    f === :light_cutoff && return Ptr{Ptr{Cfloat}}(x + 1280)
    f === :light_exponent && return Ptr{Ptr{Cfloat}}(x + 1288)
    f === :light_ambient && return Ptr{Ptr{Cfloat}}(x + 1296)
    f === :light_diffuse && return Ptr{Ptr{Cfloat}}(x + 1304)
    f === :light_specular && return Ptr{Ptr{Cfloat}}(x + 1312)
    f === :mesh_texcoordadr && return Ptr{Ptr{Cint}}(x + 1320)
    f === :mesh_graphadr && return Ptr{Ptr{Cint}}(x + 1328)
    f === :skin_matid && return Ptr{Ptr{Cint}}(x + 1336)
    f === :skin_group && return Ptr{Ptr{Cint}}(x + 1344)
    f === :skin_rgba && return Ptr{Ptr{Cfloat}}(x + 1352)
    f === :skin_inflate && return Ptr{Ptr{Cfloat}}(x + 1360)
    f === :skin_vertadr && return Ptr{Ptr{Cint}}(x + 1368)
    f === :skin_vertnum && return Ptr{Ptr{Cint}}(x + 1376)
    f === :skin_texcoordadr && return Ptr{Ptr{Cint}}(x + 1384)
    f === :skin_faceadr && return Ptr{Ptr{Cint}}(x + 1392)
    f === :skin_facenum && return Ptr{Ptr{Cint}}(x + 1400)
    f === :skin_boneadr && return Ptr{Ptr{Cint}}(x + 1408)
    f === :skin_bonenum && return Ptr{Ptr{Cint}}(x + 1416)
    f === :skin_vert && return Ptr{Ptr{Cfloat}}(x + 1424)
    f === :skin_face && return Ptr{Ptr{Cint}}(x + 1432)
    f === :skin_bonevertadr && return Ptr{Ptr{Cint}}(x + 1440)
    f === :skin_bonevertnum && return Ptr{Ptr{Cint}}(x + 1448)
    f === :skin_bonebindpos && return Ptr{Ptr{Cfloat}}(x + 1456)
    f === :skin_bonebindquat && return Ptr{Ptr{Cfloat}}(x + 1464)
    f === :skin_bonebodyid && return Ptr{Ptr{Cint}}(x + 1472)
    f === :skin_bonevertid && return Ptr{Ptr{Cint}}(x + 1480)
    f === :skin_bonevertweight && return Ptr{Ptr{Cfloat}}(x + 1488)
    f === :mat_texid && return Ptr{Ptr{Cint}}(x + 1496)
    f === :mat_texuniform && return Ptr{Ptr{mjtByte}}(x + 1504)
    f === :mat_texrepeat && return Ptr{Ptr{Cfloat}}(x + 1512)
    f === :mat_emission && return Ptr{Ptr{Cfloat}}(x + 1520)
    f === :mat_specular && return Ptr{Ptr{Cfloat}}(x + 1528)
    f === :mat_shininess && return Ptr{Ptr{Cfloat}}(x + 1536)
    f === :mat_reflectance && return Ptr{Ptr{Cfloat}}(x + 1544)
    f === :mat_rgba && return Ptr{Ptr{Cfloat}}(x + 1552)
    f === :eq_type && return Ptr{Ptr{Cint}}(x + 1560)
    f === :eq_obj1id && return Ptr{Ptr{Cint}}(x + 1568)
    f === :eq_obj2id && return Ptr{Ptr{Cint}}(x + 1576)
    f === :eq_active && return Ptr{Ptr{mjtByte}}(x + 1584)
    f === :eq_data && return Ptr{Ptr{mjtNum}}(x + 1592)
    f === :tendon_num && return Ptr{Ptr{Cint}}(x + 1600)
    f === :tendon_matid && return Ptr{Ptr{Cint}}(x + 1608)
    f === :tendon_group && return Ptr{Ptr{Cint}}(x + 1616)
    f === :tendon_limited && return Ptr{Ptr{mjtByte}}(x + 1624)
    f === :tendon_width && return Ptr{Ptr{mjtNum}}(x + 1632)
    f === :tendon_range && return Ptr{Ptr{mjtNum}}(x + 1640)
    f === :tendon_stiffness && return Ptr{Ptr{mjtNum}}(x + 1648)
    f === :tendon_damping && return Ptr{Ptr{mjtNum}}(x + 1656)
    f === :tendon_frictionloss && return Ptr{Ptr{mjtNum}}(x + 1664)
    f === :tendon_lengthspring && return Ptr{Ptr{mjtNum}}(x + 1672)
    f === :tendon_rgba && return Ptr{Ptr{Cfloat}}(x + 1680)
    f === :actuator_trntype && return Ptr{Ptr{Cint}}(x + 1688)
    f === :actuator_dyntype && return Ptr{Ptr{Cint}}(x + 1696)
    f === :actuator_trnid && return Ptr{Ptr{Cint}}(x + 1704)
    f === :actuator_actadr && return Ptr{Ptr{Cint}}(x + 1712)
    f === :actuator_actnum && return Ptr{Ptr{Cint}}(x + 1720)
    f === :actuator_group && return Ptr{Ptr{Cint}}(x + 1728)
    f === :actuator_ctrllimited && return Ptr{Ptr{mjtByte}}(x + 1736)
    f === :actuator_actlimited && return Ptr{Ptr{mjtByte}}(x + 1744)
    f === :actuator_ctrlrange && return Ptr{Ptr{mjtNum}}(x + 1752)
    f === :actuator_actrange && return Ptr{Ptr{mjtNum}}(x + 1760)
    f === :actuator_cranklength && return Ptr{Ptr{mjtNum}}(x + 1768)
    f === :sensor_type && return Ptr{Ptr{Cint}}(x + 1776)
    f === :sensor_objid && return Ptr{Ptr{Cint}}(x + 1784)
    f === :sensor_adr && return Ptr{Ptr{Cint}}(x + 1792)
    f === :name_bodyadr && return Ptr{Ptr{Cint}}(x + 1800)
    f === :name_jntadr && return Ptr{Ptr{Cint}}(x + 1808)
    f === :name_geomadr && return Ptr{Ptr{Cint}}(x + 1816)
    f === :name_siteadr && return Ptr{Ptr{Cint}}(x + 1824)
    f === :name_camadr && return Ptr{Ptr{Cint}}(x + 1832)
    f === :name_lightadr && return Ptr{Ptr{Cint}}(x + 1840)
    f === :name_eqadr && return Ptr{Ptr{Cint}}(x + 1848)
    f === :name_tendonadr && return Ptr{Ptr{Cint}}(x + 1856)
    f === :name_actuatoradr && return Ptr{Ptr{Cint}}(x + 1864)
    f === :names && return Ptr{Ptr{Cchar}}(x + 1872)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#590", f::Symbol)
    r = Ref{var"##Ctag#590"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#590"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#590"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct var"##Ctag#591"
    warning::NTuple{8, mjWarningStat}
    nefc::Cint
    ncon::Cint
    time::mjtNum
    act::Ptr{mjtNum}
    ctrl::Ptr{mjtNum}
    xfrc_applied::Ptr{mjtNum}
    sensordata::Ptr{mjtNum}
    xpos::Ptr{mjtNum}
    xquat::Ptr{mjtNum}
    xmat::Ptr{mjtNum}
    xipos::Ptr{mjtNum}
    ximat::Ptr{mjtNum}
    xanchor::Ptr{mjtNum}
    xaxis::Ptr{mjtNum}
    geom_xpos::Ptr{mjtNum}
    geom_xmat::Ptr{mjtNum}
    site_xpos::Ptr{mjtNum}
    site_xmat::Ptr{mjtNum}
    cam_xpos::Ptr{mjtNum}
    cam_xmat::Ptr{mjtNum}
    light_xpos::Ptr{mjtNum}
    light_xdir::Ptr{mjtNum}
    subtree_com::Ptr{mjtNum}
    ten_wrapadr::Ptr{Cint}
    ten_wrapnum::Ptr{Cint}
    wrap_obj::Ptr{Cint}
    wrap_xpos::Ptr{mjtNum}
    bvh_active::Ptr{mjtByte}
    contact::Ptr{mjContact}
    efc_force::Ptr{mjtNum}
end
function Base.getproperty(x::Ptr{var"##Ctag#591"}, f::Symbol)
    f === :warning && return Ptr{NTuple{8, mjWarningStat}}(x + 0)
    f === :nefc && return Ptr{Cint}(x + 64)
    f === :ncon && return Ptr{Cint}(x + 68)
    f === :time && return Ptr{mjtNum}(x + 72)
    f === :act && return Ptr{Ptr{mjtNum}}(x + 80)
    f === :ctrl && return Ptr{Ptr{mjtNum}}(x + 88)
    f === :xfrc_applied && return Ptr{Ptr{mjtNum}}(x + 96)
    f === :sensordata && return Ptr{Ptr{mjtNum}}(x + 104)
    f === :xpos && return Ptr{Ptr{mjtNum}}(x + 112)
    f === :xquat && return Ptr{Ptr{mjtNum}}(x + 120)
    f === :xmat && return Ptr{Ptr{mjtNum}}(x + 128)
    f === :xipos && return Ptr{Ptr{mjtNum}}(x + 136)
    f === :ximat && return Ptr{Ptr{mjtNum}}(x + 144)
    f === :xanchor && return Ptr{Ptr{mjtNum}}(x + 152)
    f === :xaxis && return Ptr{Ptr{mjtNum}}(x + 160)
    f === :geom_xpos && return Ptr{Ptr{mjtNum}}(x + 168)
    f === :geom_xmat && return Ptr{Ptr{mjtNum}}(x + 176)
    f === :site_xpos && return Ptr{Ptr{mjtNum}}(x + 184)
    f === :site_xmat && return Ptr{Ptr{mjtNum}}(x + 192)
    f === :cam_xpos && return Ptr{Ptr{mjtNum}}(x + 200)
    f === :cam_xmat && return Ptr{Ptr{mjtNum}}(x + 208)
    f === :light_xpos && return Ptr{Ptr{mjtNum}}(x + 216)
    f === :light_xdir && return Ptr{Ptr{mjtNum}}(x + 224)
    f === :subtree_com && return Ptr{Ptr{mjtNum}}(x + 232)
    f === :ten_wrapadr && return Ptr{Ptr{Cint}}(x + 240)
    f === :ten_wrapnum && return Ptr{Ptr{Cint}}(x + 248)
    f === :wrap_obj && return Ptr{Ptr{Cint}}(x + 256)
    f === :wrap_xpos && return Ptr{Ptr{mjtNum}}(x + 264)
    f === :bvh_active && return Ptr{Ptr{mjtByte}}(x + 272)
    f === :contact && return Ptr{Ptr{mjContact}}(x + 280)
    f === :efc_force && return Ptr{Ptr{mjtNum}}(x + 288)
    return getfield(x, f)
end

function Base.getproperty(x::var"##Ctag#591", f::Symbol)
    r = Ref{var"##Ctag#591"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#591"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{var"##Ctag#591"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct mjvSceneState_
    data::NTuple{10856, UInt8}
end

function Base.getproperty(x::Ptr{mjvSceneState_}, f::Symbol)
    f === :nbuffer && return Ptr{Cint}(x + 0)
    f === :buffer && return Ptr{Ptr{Cvoid}}(x + 8)
    f === :maxgeom && return Ptr{Cint}(x + 16)
    f === :plugincache && return Ptr{mjvScene}(x + 24)
    f === :model && return Ptr{var"##Ctag#590"}(x + 8680)
    f === :data && return Ptr{var"##Ctag#591"}(x + 10560)
    return getfield(x, f)
end

function Base.getproperty(x::mjvSceneState_, f::Symbol)
    r = Ref{mjvSceneState_}(x)
    ptr = Base.unsafe_convert(Ptr{mjvSceneState_}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{mjvSceneState_}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const mjvSceneState = mjvSceneState_

function mj_defaultVFS(vfs)
    ccall((:mj_defaultVFS, libmujuco), Cvoid, (Ptr{mjVFS},), vfs)
end

function mj_addFileVFS(vfs, directory, filename)
    ccall((:mj_addFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}, Ptr{Cchar}), vfs, directory, filename)
end

function mj_makeEmptyFileVFS(vfs, filename, filesize)
    ccall((:mj_makeEmptyFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}, Cint), vfs, filename, filesize)
end

function mj_findFileVFS(vfs, filename)
    ccall((:mj_findFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}), vfs, filename)
end

function mj_deleteFileVFS(vfs, filename)
    ccall((:mj_deleteFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}), vfs, filename)
end

function mj_deleteVFS(vfs)
    ccall((:mj_deleteVFS, libmujuco), Cvoid, (Ptr{mjVFS},), vfs)
end

function mj_loadXML(filename, vfs, error, error_sz)
    ccall((:mj_loadXML, libmujuco), Ptr{mjModel}, (Ptr{Cchar}, Ptr{mjVFS}, Ptr{Cchar}, Cint), filename, vfs, error, error_sz)
end

function mj_saveLastXML(filename, m, error, error_sz)
    ccall((:mj_saveLastXML, libmujuco), Cint, (Ptr{Cchar}, Ptr{mjModel}, Ptr{Cchar}, Cint), filename, m, error, error_sz)
end

function mj_freeLastXML()
    ccall((:mj_freeLastXML, libmujuco), Cvoid, ())
end

function mj_printSchema(filename, buffer, buffer_sz, flg_html, flg_pad)
    ccall((:mj_printSchema, libmujuco), Cint, (Ptr{Cchar}, Ptr{Cchar}, Cint, Cint, Cint), filename, buffer, buffer_sz, flg_html, flg_pad)
end

function mj_step(m, d)
    ccall((:mj_step, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_step1(m, d)
    ccall((:mj_step1, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_step2(m, d)
    ccall((:mj_step2, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_forward(m, d)
    ccall((:mj_forward, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_inverse(m, d)
    ccall((:mj_inverse, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_forwardSkip(m, d, skipstage, skipsensor)
    ccall((:mj_forwardSkip, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint), m, d, skipstage, skipsensor)
end

function mj_inverseSkip(m, d, skipstage, skipsensor)
    ccall((:mj_inverseSkip, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint), m, d, skipstage, skipsensor)
end

function mj_defaultLROpt(opt)
    ccall((:mj_defaultLROpt, libmujuco), Cvoid, (Ptr{mjLROpt},), opt)
end

function mj_defaultSolRefImp(solref, solimp)
    ccall((:mj_defaultSolRefImp, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), solref, solimp)
end

function mj_defaultOption(opt)
    ccall((:mj_defaultOption, libmujuco), Cvoid, (Ptr{mjOption},), opt)
end

function mj_defaultVisual(vis)
    ccall((:mj_defaultVisual, libmujuco), Cvoid, (Ptr{mjVisual},), vis)
end

function mj_copyModel(dest, src)
    ccall((:mj_copyModel, libmujuco), Ptr{mjModel}, (Ptr{mjModel}, Ptr{mjModel}), dest, src)
end

function mj_saveModel(m, filename, buffer, buffer_sz)
    ccall((:mj_saveModel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}, Ptr{Cvoid}, Cint), m, filename, buffer, buffer_sz)
end

function mj_loadModel(filename, vfs)
    ccall((:mj_loadModel, libmujuco), Ptr{mjModel}, (Ptr{Cchar}, Ptr{mjVFS}), filename, vfs)
end

function mj_deleteModel(m)
    ccall((:mj_deleteModel, libmujuco), Cvoid, (Ptr{mjModel},), m)
end

function mj_sizeModel(m)
    ccall((:mj_sizeModel, libmujuco), Cint, (Ptr{mjModel},), m)
end

function mj_makeData(m)
    ccall((:mj_makeData, libmujuco), Ptr{mjData}, (Ptr{mjModel},), m)
end

function mj_copyData(dest, m, src)
    ccall((:mj_copyData, libmujuco), Ptr{mjData}, (Ptr{mjData}, Ptr{mjModel}, Ptr{mjData}), dest, m, src)
end

function mj_resetData(m, d)
    ccall((:mj_resetData, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_resetDataDebug(m, d, debug_value)
    ccall((:mj_resetDataDebug, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cuchar), m, d, debug_value)
end

function mj_resetDataKeyframe(m, d, key)
    ccall((:mj_resetDataKeyframe, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, key)
end

function mj_stackAlloc(d, size)
    ccall((:mj_stackAlloc, libmujuco), Ptr{mjtNum}, (Ptr{mjData}, Cint), d, size)
end

function mj_stackAllocInt(d, size)
    ccall((:mj_stackAllocInt, libmujuco), Ptr{Cint}, (Ptr{mjData}, Cint), d, size)
end

function mj_deleteData(d)
    ccall((:mj_deleteData, libmujuco), Cvoid, (Ptr{mjData},), d)
end

function mj_resetCallbacks()
    ccall((:mj_resetCallbacks, libmujuco), Cvoid, ())
end

function mj_setConst(m, d)
    ccall((:mj_setConst, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_setLengthRange(m, d, index, opt, error, error_sz)
    ccall((:mj_setLengthRange, libmujuco), Cint, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjLROpt}, Ptr{Cchar}, Cint), m, d, index, opt, error, error_sz)
end

function mj_printFormattedModel(m, filename, float_format)
    ccall((:mj_printFormattedModel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}, Ptr{Cchar}), m, filename, float_format)
end

function mj_printModel(m, filename)
    ccall((:mj_printModel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}), m, filename)
end

function mj_printFormattedData(m, d, filename, float_format)
    ccall((:mj_printFormattedData, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{Cchar}, Ptr{Cchar}), m, d, filename, float_format)
end

function mj_printData(m, d, filename)
    ccall((:mj_printData, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{Cchar}), m, d, filename)
end

function mju_printMat(mat, nr, nc)
    ccall((:mju_printMat, libmujuco), Cvoid, (Ptr{mjtNum}, Cint, Cint), mat, nr, nc)
end

function mju_printMatSparse(mat, nr, rownnz, rowadr, colind)
    ccall((:mju_printMatSparse, libmujuco), Cvoid, (Ptr{mjtNum}, Cint, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), mat, nr, rownnz, rowadr, colind)
end

function mj_fwdPosition(m, d)
    ccall((:mj_fwdPosition, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_fwdVelocity(m, d)
    ccall((:mj_fwdVelocity, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_fwdActuation(m, d)
    ccall((:mj_fwdActuation, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_fwdAcceleration(m, d)
    ccall((:mj_fwdAcceleration, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_fwdConstraint(m, d)
    ccall((:mj_fwdConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_Euler(m, d)
    ccall((:mj_Euler, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_RungeKutta(m, d, N)
    ccall((:mj_RungeKutta, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, N)
end

function mj_invPosition(m, d)
    ccall((:mj_invPosition, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_invVelocity(m, d)
    ccall((:mj_invVelocity, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_invConstraint(m, d)
    ccall((:mj_invConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_compareFwdInv(m, d)
    ccall((:mj_compareFwdInv, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_sensorPos(m, d)
    ccall((:mj_sensorPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_sensorVel(m, d)
    ccall((:mj_sensorVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_sensorAcc(m, d)
    ccall((:mj_sensorAcc, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_energyPos(m, d)
    ccall((:mj_energyPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_energyVel(m, d)
    ccall((:mj_energyVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_checkPos(m, d)
    ccall((:mj_checkPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_checkVel(m, d)
    ccall((:mj_checkVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_checkAcc(m, d)
    ccall((:mj_checkAcc, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_kinematics(m, d)
    ccall((:mj_kinematics, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_comPos(m, d)
    ccall((:mj_comPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_camlight(m, d)
    ccall((:mj_camlight, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_tendon(m, d)
    ccall((:mj_tendon, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_transmission(m, d)
    ccall((:mj_transmission, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_crb(m, d)
    ccall((:mj_crb, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_factorM(m, d)
    ccall((:mj_factorM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_solveM(m, d, x, y, n)
    ccall((:mj_solveM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, x, y, n)
end

function mj_solveM2(m, d, x, y, n)
    ccall((:mj_solveM2, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, x, y, n)
end

function mj_comVel(m, d)
    ccall((:mj_comVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_passive(m, d)
    ccall((:mj_passive, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_subtreeVel(m, d)
    ccall((:mj_subtreeVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_rne(m, d, flg_acc, result)
    ccall((:mj_rne, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}), m, d, flg_acc, result)
end

function mj_rnePostConstraint(m, d)
    ccall((:mj_rnePostConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_collision(m, d)
    ccall((:mj_collision, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_makeConstraint(m, d)
    ccall((:mj_makeConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_projectConstraint(m, d)
    ccall((:mj_projectConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_referenceConstraint(m, d)
    ccall((:mj_referenceConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end

function mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)
    ccall((:mj_constraintUpdate, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jar, cost, flg_coneHessian)
end

function mj_stateSize(m, spec)
    ccall((:mj_stateSize, libmujuco), Cint, (Ptr{mjModel}, Cuint), m, spec)
end

function mj_getState(m, d, state, spec)
    ccall((:mj_getState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cuint), m, d, state, spec)
end

function mj_setState(m, d, state, spec)
    ccall((:mj_setState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cuint), m, d, state, spec)
end

function mj_addContact(m, d, con)
    ccall((:mj_addContact, libmujuco), Cint, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjContact}), m, d, con)
end

function mj_isPyramidal(m)
    ccall((:mj_isPyramidal, libmujuco), Cint, (Ptr{mjModel},), m)
end

function mj_isSparse(m)
    ccall((:mj_isSparse, libmujuco), Cint, (Ptr{mjModel},), m)
end

function mj_isDual(m)
    ccall((:mj_isDual, libmujuco), Cint, (Ptr{mjModel},), m)
end

function mj_mulJacVec(m, d, res, vec)
    ccall((:mj_mulJacVec, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end

function mj_mulJacTVec(m, d, res, vec)
    ccall((:mj_mulJacTVec, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end

function mj_jac(m, d, jacp, jacr, point, body)
    ccall((:mj_jac, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, point, body)
end

function mj_jacBody(m, d, jacp, jacr, body)
    ccall((:mj_jacBody, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, body)
end

function mj_jacBodyCom(m, d, jacp, jacr, body)
    ccall((:mj_jacBodyCom, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, body)
end

function mj_jacSubtreeCom(m, d, jacp, body)
    ccall((:mj_jacSubtreeCom, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cint), m, d, jacp, body)
end

function mj_jacGeom(m, d, jacp, jacr, geom)
    ccall((:mj_jacGeom, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, geom)
end

function mj_jacSite(m, d, jacp, jacr, site)
    ccall((:mj_jacSite, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, site)
end

function mj_jacPointAxis(m, d, jacPoint, jacAxis, point, axis, body)
    ccall((:mj_jacPointAxis, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacPoint, jacAxis, point, axis, body)
end

function mj_name2id(m, type, name)
    ccall((:mj_name2id, libmujuco), Cint, (Ptr{mjModel}, Cint, Ptr{Cchar}), m, type, name)
end

function mj_id2name(m, type, id)
    ccall((:mj_id2name, libmujuco), Ptr{Cchar}, (Ptr{mjModel}, Cint, Cint), m, type, id)
end

function mj_fullM(m, dst, M)
    ccall((:mj_fullM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}), m, dst, M)
end

function mj_mulM(m, d, res, vec)
    ccall((:mj_mulM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end

function mj_mulM2(m, d, res, vec)
    ccall((:mj_mulM2, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end

function mj_addM(m, d, dst, rownnz, rowadr, colind)
    ccall((:mj_addM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), m, d, dst, rownnz, rowadr, colind)
end

function mj_applyFT(m, d, force, torque, point, body, qfrc_target)
    ccall((:mj_applyFT, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}), m, d, force, torque, point, body, qfrc_target)
end

function mj_objectVelocity(m, d, objtype, objid, res, flg_local)
    ccall((:mj_objectVelocity, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint), m, d, objtype, objid, res, flg_local)
end

function mj_objectAcceleration(m, d, objtype, objid, res, flg_local)
    ccall((:mj_objectAcceleration, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint), m, d, objtype, objid, res, flg_local)
end

function mj_contactForce(m, d, id, result)
    ccall((:mj_contactForce, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}), m, d, id, result)
end

function mj_differentiatePos(m, qvel, dt, qpos1, qpos2)
    ccall((:mj_differentiatePos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}), m, qvel, dt, qpos1, qpos2)
end

function mj_integratePos(m, qpos, qvel, dt)
    ccall((:mj_integratePos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), m, qpos, qvel, dt)
end

function mj_normalizeQuat(m, qpos)
    ccall((:mj_normalizeQuat, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}), m, qpos)
end

function mj_local2Global(d, xpos, xmat, pos, quat, body, sameframe)
    ccall((:mj_local2Global, libmujuco), Cvoid, (Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, mjtByte), d, xpos, xmat, pos, quat, body, sameframe)
end

function mj_getTotalmass(m)
    ccall((:mj_getTotalmass, libmujuco), mjtNum, (Ptr{mjModel},), m)
end

function mj_setTotalmass(m, newmass)
    ccall((:mj_setTotalmass, libmujuco), Cvoid, (Ptr{mjModel}, mjtNum), m, newmass)
end

function mj_getPluginConfig(m, plugin_id, attrib)
    ccall((:mj_getPluginConfig, libmujuco), Ptr{Cchar}, (Ptr{mjModel}, Cint, Ptr{Cchar}), m, plugin_id, attrib)
end

function mj_loadPluginLibrary(path)
    ccall((:mj_loadPluginLibrary, libmujuco), Cvoid, (Ptr{Cchar},), path)
end

function mj_loadAllPluginLibraries(directory, callback)
    ccall((:mj_loadAllPluginLibraries, libmujuco), Cvoid, (Ptr{Cchar}, mjfPluginLibraryLoadCallback), directory, callback)
end

function mj_version()
    ccall((:mj_version, libmujuco), Cint, ())
end

# no prototype is found for this function at mujoco.h:480:19, please use with caution
function mj_versionString()
    ccall((:mj_versionString, libmujuco), Ptr{Cchar}, ())
end

function mj_multiRay(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid, dist, nray, cutoff)
    ccall((:mj_multiRay, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtByte}, mjtByte, Cint, Ptr{Cint}, Ptr{mjtNum}, Cint, mjtNum), m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid, dist, nray, cutoff)
end

function mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)
    ccall((:mj_ray, libmujuco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtByte}, mjtByte, Cint, Ptr{Cint}), m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)
end

function mj_rayHfield(m, d, geomid, pnt, vec)
    ccall((:mj_rayHfield, libmujuco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), m, d, geomid, pnt, vec)
end

function mj_rayMesh(m, d, geomid, pnt, vec)
    ccall((:mj_rayMesh, libmujuco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), m, d, geomid, pnt, vec)
end

function mju_rayGeom(pos, mat, size, pnt, vec, geomtype)
    ccall((:mju_rayGeom, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), pos, mat, size, pnt, vec, geomtype)
end

function mju_raySkin(nface, nvert, face, vert, pnt, vec, vertid)
    ccall((:mju_raySkin, libmujuco), mjtNum, (Cint, Cint, Ptr{Cint}, Ptr{Cfloat}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}), nface, nvert, face, vert, pnt, vec, vertid)
end

function mjv_defaultCamera(cam)
    ccall((:mjv_defaultCamera, libmujuco), Cvoid, (Ptr{mjvCamera},), cam)
end

function mjv_defaultFreeCamera(m, cam)
    ccall((:mjv_defaultFreeCamera, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjvCamera}), m, cam)
end

function mjv_defaultPerturb(pert)
    ccall((:mjv_defaultPerturb, libmujuco), Cvoid, (Ptr{mjvPerturb},), pert)
end

function mjv_room2model(modelpos, modelquat, roompos, roomquat, scn)
    ccall((:mjv_room2model, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), modelpos, modelquat, roompos, roomquat, scn)
end

function mjv_model2room(roompos, roomquat, modelpos, modelquat, scn)
    ccall((:mjv_model2room, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), roompos, roomquat, modelpos, modelquat, scn)
end

function mjv_cameraInModel(headpos, forward, up, scn)
    ccall((:mjv_cameraInModel, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), headpos, forward, up, scn)
end

function mjv_cameraInRoom(headpos, forward, up, scn)
    ccall((:mjv_cameraInRoom, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), headpos, forward, up, scn)
end

function mjv_frustumHeight(scn)
    ccall((:mjv_frustumHeight, libmujuco), mjtNum, (Ptr{mjvScene},), scn)
end

function mjv_alignToCamera(res, vec, forward)
    ccall((:mjv_alignToCamera, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, forward)
end

function mjv_moveCamera(m, action, reldx, reldy, scn, cam)
    ccall((:mjv_moveCamera, libmujuco), Cvoid, (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}), m, action, reldx, reldy, scn, cam)
end

function mjv_moveCameraFromState(scnstate, action, reldx, reldy, scn, cam)
    ccall((:mjv_moveCameraFromState, libmujuco), Cvoid, (Ptr{mjvSceneState}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}), scnstate, action, reldx, reldy, scn, cam)
end

function mjv_movePerturb(m, d, action, reldx, reldy, scn, pert)
    ccall((:mjv_movePerturb, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}), m, d, action, reldx, reldy, scn, pert)
end

function mjv_movePerturbFromState(scnstate, action, reldx, reldy, scn, pert)
    ccall((:mjv_movePerturbFromState, libmujuco), Cvoid, (Ptr{mjvSceneState}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}), scnstate, action, reldx, reldy, scn, pert)
end

function mjv_moveModel(m, action, reldx, reldy, roomup, scn)
    ccall((:mjv_moveModel, libmujuco), Cvoid, (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjtNum}, Ptr{mjvScene}), m, action, reldx, reldy, roomup, scn)
end

function mjv_initPerturb(m, d, scn, pert)
    ccall((:mjv_initPerturb, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}, Ptr{mjvPerturb}), m, d, scn, pert)
end

function mjv_applyPerturbPose(m, d, pert, flg_paused)
    ccall((:mjv_applyPerturbPose, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}, Cint), m, d, pert, flg_paused)
end

function mjv_applyPerturbForce(m, d, pert)
    ccall((:mjv_applyPerturbForce, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}), m, d, pert)
end

function mjv_averageCamera(cam1, cam2)
    ccall((:mjv_averageCamera, libmujuco), mjvGLCamera, (Ptr{mjvGLCamera}, Ptr{mjvGLCamera}), cam1, cam2)
end

function mjv_select(m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid)
    ccall((:mjv_select, libmujuco), Cint, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, mjtNum, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}), m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid)
end

function mjv_defaultOption(opt)
    ccall((:mjv_defaultOption, libmujuco), Cvoid, (Ptr{mjvOption},), opt)
end

function mjv_defaultFigure(fig)
    ccall((:mjv_defaultFigure, libmujuco), Cvoid, (Ptr{mjvFigure},), fig)
end

function mjv_initGeom(geom, type, size, pos, mat, rgba)
    ccall((:mjv_initGeom, libmujuco), Cvoid, (Ptr{mjvGeom}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cfloat}), geom, type, size, pos, mat, rgba)
end

function mjv_makeConnector(geom, type, width, a0, a1, a2, b0, b1, b2)
    ccall((:mjv_makeConnector, libmujuco), Cvoid, (Ptr{mjvGeom}, Cint, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum), geom, type, width, a0, a1, a2, b0, b1, b2)
end

function mjv_connector(geom, type, width, from, to)
    ccall((:mjv_connector, libmujuco), Cvoid, (Ptr{mjvGeom}, Cint, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}), geom, type, width, from, to)
end

function mjv_defaultScene(scn)
    ccall((:mjv_defaultScene, libmujuco), Cvoid, (Ptr{mjvScene},), scn)
end

function mjv_makeScene(m, scn, maxgeom)
    ccall((:mjv_makeScene, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjvScene}, Cint), m, scn, maxgeom)
end

function mjv_freeScene(scn)
    ccall((:mjv_freeScene, libmujuco), Cvoid, (Ptr{mjvScene},), scn)
end

function mjv_updateScene(m, d, opt, pert, cam, catmask, scn)
    ccall((:mjv_updateScene, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Ptr{mjvCamera}, Cint, Ptr{mjvScene}), m, d, opt, pert, cam, catmask, scn)
end

function mjv_updateSceneFromState(scnstate, opt, pert, cam, catmask, scn)
    ccall((:mjv_updateSceneFromState, libmujuco), Cint, (Ptr{mjvSceneState}, Ptr{mjvOption}, Ptr{mjvPerturb}, Ptr{mjvCamera}, Cint, Ptr{mjvScene}), scnstate, opt, pert, cam, catmask, scn)
end

function mjv_defaultSceneState(scnstate)
    ccall((:mjv_defaultSceneState, libmujuco), Cvoid, (Ptr{mjvSceneState},), scnstate)
end

function mjv_makeSceneState(m, d, scnstate, maxgeom)
    ccall((:mjv_makeSceneState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvSceneState}, Cint), m, d, scnstate, maxgeom)
end

function mjv_freeSceneState(scnstate)
    ccall((:mjv_freeSceneState, libmujuco), Cvoid, (Ptr{mjvSceneState},), scnstate)
end

function mjv_updateSceneState(m, d, opt, scnstate)
    ccall((:mjv_updateSceneState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvSceneState}), m, d, opt, scnstate)
end

function mjv_addGeoms(m, d, opt, pert, catmask, scn)
    ccall((:mjv_addGeoms, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Cint, Ptr{mjvScene}), m, d, opt, pert, catmask, scn)
end

function mjv_makeLights(m, d, scn)
    ccall((:mjv_makeLights, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}), m, d, scn)
end

function mjv_updateCamera(m, d, cam, scn)
    ccall((:mjv_updateCamera, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvCamera}, Ptr{mjvScene}), m, d, cam, scn)
end

function mjv_updateSkin(m, d, scn)
    ccall((:mjv_updateSkin, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}), m, d, scn)
end

function mjr_defaultContext(con)
    ccall((:mjr_defaultContext, libmujuco), Cvoid, (Ptr{mjrContext},), con)
end

function mjr_makeContext(m, con, fontscale)
    ccall((:mjr_makeContext, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, fontscale)
end

function mjr_changeFont(fontscale, con)
    ccall((:mjr_changeFont, libmujuco), Cvoid, (Cint, Ptr{mjrContext}), fontscale, con)
end

function mjr_addAux(index, width, height, samples, con)
    ccall((:mjr_addAux, libmujuco), Cvoid, (Cint, Cint, Cint, Cint, Ptr{mjrContext}), index, width, height, samples, con)
end

function mjr_freeContext(con)
    ccall((:mjr_freeContext, libmujuco), Cvoid, (Ptr{mjrContext},), con)
end

function mjr_resizeOffscreen(width, height, con)
    ccall((:mjr_resizeOffscreen, libmujuco), Cvoid, (Cint, Cint, Ptr{mjrContext}), width, height, con)
end

function mjr_uploadTexture(m, con, texid)
    ccall((:mjr_uploadTexture, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, texid)
end

function mjr_uploadMesh(m, con, meshid)
    ccall((:mjr_uploadMesh, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, meshid)
end

function mjr_uploadHField(m, con, hfieldid)
    ccall((:mjr_uploadHField, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, hfieldid)
end

function mjr_restoreBuffer(con)
    ccall((:mjr_restoreBuffer, libmujuco), Cvoid, (Ptr{mjrContext},), con)
end

function mjr_setBuffer(framebuffer, con)
    ccall((:mjr_setBuffer, libmujuco), Cvoid, (Cint, Ptr{mjrContext}), framebuffer, con)
end

function mjr_readPixels(rgb, depth, viewport, con)
    ccall((:mjr_readPixels, libmujuco), Cvoid, (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}), rgb, depth, viewport, con)
end

function mjr_drawPixels(rgb, depth, viewport, con)
    ccall((:mjr_drawPixels, libmujuco), Cvoid, (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}), rgb, depth, viewport, con)
end

function mjr_blitBuffer(src, dst, flg_color, flg_depth, con)
    ccall((:mjr_blitBuffer, libmujuco), Cvoid, (mjrRect, mjrRect, Cint, Cint, Ptr{mjrContext}), src, dst, flg_color, flg_depth, con)
end

function mjr_setAux(index, con)
    ccall((:mjr_setAux, libmujuco), Cvoid, (Cint, Ptr{mjrContext}), index, con)
end

function mjr_blitAux(index, src, left, bottom, con)
    ccall((:mjr_blitAux, libmujuco), Cvoid, (Cint, mjrRect, Cint, Cint, Ptr{mjrContext}), index, src, left, bottom, con)
end

function mjr_text(font, txt, con, x, y, r, g, b)
    ccall((:mjr_text, libmujuco), Cvoid, (Cint, Ptr{Cchar}, Ptr{mjrContext}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat), font, txt, con, x, y, r, g, b)
end

function mjr_overlay(font, gridpos, viewport, overlay, overlay2, con)
    ccall((:mjr_overlay, libmujuco), Cvoid, (Cint, Cint, mjrRect, Ptr{Cchar}, Ptr{Cchar}, Ptr{mjrContext}), font, gridpos, viewport, overlay, overlay2, con)
end

function mjr_maxViewport(con)
    ccall((:mjr_maxViewport, libmujuco), mjrRect, (Ptr{mjrContext},), con)
end

function mjr_rectangle(viewport, r, g, b, a)
    ccall((:mjr_rectangle, libmujuco), Cvoid, (mjrRect, Cfloat, Cfloat, Cfloat, Cfloat), viewport, r, g, b, a)
end

function mjr_label(viewport, font, txt, r, g, b, a, rt, gt, bt, con)
    ccall((:mjr_label, libmujuco), Cvoid, (mjrRect, Cint, Ptr{Cchar}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Ptr{mjrContext}), viewport, font, txt, r, g, b, a, rt, gt, bt, con)
end

function mjr_figure(viewport, fig, con)
    ccall((:mjr_figure, libmujuco), Cvoid, (mjrRect, Ptr{mjvFigure}, Ptr{mjrContext}), viewport, fig, con)
end

function mjr_render(viewport, scn, con)
    ccall((:mjr_render, libmujuco), Cvoid, (mjrRect, Ptr{mjvScene}, Ptr{mjrContext}), viewport, scn, con)
end

function mjr_finish()
    ccall((:mjr_finish, libmujuco), Cvoid, ())
end

function mjr_getError()
    ccall((:mjr_getError, libmujuco), Cint, ())
end

function mjr_findRect(x, y, nrect, rect)
    ccall((:mjr_findRect, libmujuco), Cint, (Cint, Cint, Cint, Ptr{mjrRect}), x, y, nrect, rect)
end

function mjui_themeSpacing(ind)
    ccall((:mjui_themeSpacing, libmujuco), mjuiThemeSpacing, (Cint,), ind)
end

function mjui_themeColor(ind)
    ccall((:mjui_themeColor, libmujuco), mjuiThemeColor, (Cint,), ind)
end

function mjui_add(ui, def)
    ccall((:mjui_add, libmujuco), Cvoid, (Ptr{mjUI}, Ptr{mjuiDef}), ui, def)
end

function mjui_addToSection(ui, sect, def)
    ccall((:mjui_addToSection, libmujuco), Cvoid, (Ptr{mjUI}, Cint, Ptr{mjuiDef}), ui, sect, def)
end

function mjui_resize(ui, con)
    ccall((:mjui_resize, libmujuco), Cvoid, (Ptr{mjUI}, Ptr{mjrContext}), ui, con)
end

function mjui_update(section, item, ui, state, con)
    ccall((:mjui_update, libmujuco), Cvoid, (Cint, Cint, Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), section, item, ui, state, con)
end

function mjui_event(ui, state, con)
    ccall((:mjui_event, libmujuco), Ptr{mjuiItem}, (Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), ui, state, con)
end

function mjui_render(ui, state, con)
    ccall((:mjui_render, libmujuco), Cvoid, (Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), ui, state, con)
end

function mju_error_i(msg, i)
    ccall((:mju_error_i, libmujuco), Cvoid, (Ptr{Cchar}, Cint), msg, i)
end

function mju_error_s(msg, text)
    ccall((:mju_error_s, libmujuco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), msg, text)
end

function mju_warning_i(msg, i)
    ccall((:mju_warning_i, libmujuco), Cvoid, (Ptr{Cchar}, Cint), msg, i)
end

function mju_warning_s(msg, text)
    ccall((:mju_warning_s, libmujuco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), msg, text)
end

function mju_clearHandlers()
    ccall((:mju_clearHandlers, libmujuco), Cvoid, ())
end

function mju_malloc(size)
    ccall((:mju_malloc, libmujuco), Ptr{Cvoid}, (Csize_t,), size)
end

function mju_free(ptr)
    ccall((:mju_free, libmujuco), Cvoid, (Ptr{Cvoid},), ptr)
end

function mj_warning(d, warning, info)
    ccall((:mj_warning, libmujuco), Cvoid, (Ptr{mjData}, Cint, Cint), d, warning, info)
end

function mju_writeLog(type, msg)
    ccall((:mju_writeLog, libmujuco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), type, msg)
end

function mju_zero3(res)
    ccall((:mju_zero3, libmujuco), Cvoid, (Ptr{mjtNum},), res)
end

function mju_copy3(res, data)
    ccall((:mju_copy3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, data)
end

function mju_scl3(res, vec, scl)
    ccall((:mju_scl3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec, scl)
end

function mju_add3(res, vec1, vec2)
    ccall((:mju_add3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec1, vec2)
end

function mju_sub3(res, vec1, vec2)
    ccall((:mju_sub3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec1, vec2)
end

function mju_addTo3(res, vec)
    ccall((:mju_addTo3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, vec)
end

function mju_subFrom3(res, vec)
    ccall((:mju_subFrom3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, vec)
end

function mju_addToScl3(res, vec, scl)
    ccall((:mju_addToScl3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec, scl)
end

function mju_addScl3(res, vec1, vec2, scl)
    ccall((:mju_addScl3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec1, vec2, scl)
end

function mju_normalize3(res)
    ccall((:mju_normalize3, libmujuco), mjtNum, (Ptr{mjtNum},), res)
end

function mju_norm3(vec)
    ccall((:mju_norm3, libmujuco), mjtNum, (Ptr{mjtNum},), vec)
end

function mju_dot3(vec1, vec2)
    ccall((:mju_dot3, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}), vec1, vec2)
end

function mju_dist3(pos1, pos2)
    ccall((:mju_dist3, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}), pos1, pos2)
end

function mju_rotVecMat(res, vec, mat)
    ccall((:mju_rotVecMat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, mat)
end

function mju_rotVecMatT(res, vec, mat)
    ccall((:mju_rotVecMatT, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, mat)
end

function mju_cross(res, a, b)
    ccall((:mju_cross, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, a, b)
end

function mju_zero4(res)
    ccall((:mju_zero4, libmujuco), Cvoid, (Ptr{mjtNum},), res)
end

function mju_unit4(res)
    ccall((:mju_unit4, libmujuco), Cvoid, (Ptr{mjtNum},), res)
end

function mju_copy4(res, data)
    ccall((:mju_copy4, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, data)
end

function mju_normalize4(res)
    ccall((:mju_normalize4, libmujuco), mjtNum, (Ptr{mjtNum},), res)
end

function mju_zero(res, n)
    ccall((:mju_zero, libmujuco), Cvoid, (Ptr{mjtNum}, Cint), res, n)
end

function mju_fill(res, val, n)
    ccall((:mju_fill, libmujuco), Cvoid, (Ptr{mjtNum}, mjtNum, Cint), res, val, n)
end

function mju_copy(res, data, n)
    ccall((:mju_copy, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, data, n)
end

function mju_sum(vec, n)
    ccall((:mju_sum, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), vec, n)
end

function mju_L1(vec, n)
    ccall((:mju_L1, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), vec, n)
end

function mju_scl(res, vec, scl, n)
    ccall((:mju_scl, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec, scl, n)
end

function mju_add(res, vec1, vec2, n)
    ccall((:mju_add, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec1, vec2, n)
end

function mju_sub(res, vec1, vec2, n)
    ccall((:mju_sub, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec1, vec2, n)
end

function mju_addTo(res, vec, n)
    ccall((:mju_addTo, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec, n)
end

function mju_subFrom(res, vec, n)
    ccall((:mju_subFrom, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec, n)
end

function mju_addToScl(res, vec, scl, n)
    ccall((:mju_addToScl, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec, scl, n)
end

function mju_addScl(res, vec1, vec2, scl, n)
    ccall((:mju_addScl, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec1, vec2, scl, n)
end

function mju_normalize(res, n)
    ccall((:mju_normalize, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), res, n)
end

function mju_norm(res, n)
    ccall((:mju_norm, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), res, n)
end

function mju_dot(vec1, vec2, n)
    ccall((:mju_dot, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), vec1, vec2, n)
end

function mju_mulMatVec(res, mat, vec, nr, nc)
    ccall((:mju_mulMatVec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, vec, nr, nc)
end

function mju_mulMatTVec(res, mat, vec, nr, nc)
    ccall((:mju_mulMatTVec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, vec, nr, nc)
end

function mju_mulVecMatVec(vec1, mat, vec2, n)
    ccall((:mju_mulVecMatVec, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), vec1, mat, vec2, n)
end

function mju_transpose(res, mat, nr, nc)
    ccall((:mju_transpose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, nr, nc)
end

function mju_symmetrize(res, mat, n)
    ccall((:mju_symmetrize, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, mat, n)
end

function mju_eye(mat, n)
    ccall((:mju_eye, libmujuco), Cvoid, (Ptr{mjtNum}, Cint), mat, n)
end

function mju_mulMatMat(res, mat1, mat2, r1, c1, c2)
    ccall((:mju_mulMatMat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, c2)
end

function mju_mulMatMatT(res, mat1, mat2, r1, c1, r2)
    ccall((:mju_mulMatMatT, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, r2)
end

function mju_mulMatTMat(res, mat1, mat2, r1, c1, c2)
    ccall((:mju_mulMatTMat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, c2)
end

function mju_sqrMatTD(res, mat, diag, nr, nc)
    ccall((:mju_sqrMatTD, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, diag, nr, nc)
end

function mju_transformSpatial(res, vec, flg_force, newpos, oldpos, rotnew2old)
    ccall((:mju_transformSpatial, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, flg_force, newpos, oldpos, rotnew2old)
end

function mju_rotVecQuat(res, vec, quat)
    ccall((:mju_rotVecQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, quat)
end

function mju_negQuat(res, quat)
    ccall((:mju_negQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, quat)
end

function mju_mulQuat(res, quat1, quat2)
    ccall((:mju_mulQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat1, quat2)
end

function mju_mulQuatAxis(res, quat, axis)
    ccall((:mju_mulQuatAxis, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat, axis)
end

function mju_axisAngle2Quat(res, axis, angle)
    ccall((:mju_axisAngle2Quat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, axis, angle)
end

function mju_quat2Vel(res, quat, dt)
    ccall((:mju_quat2Vel, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, quat, dt)
end

function mju_subQuat(res, qa, qb)
    ccall((:mju_subQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, qa, qb)
end

function mju_quat2Mat(res, quat)
    ccall((:mju_quat2Mat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, quat)
end

function mju_mat2Quat(quat, mat)
    ccall((:mju_mat2Quat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), quat, mat)
end

function mju_derivQuat(res, quat, vel)
    ccall((:mju_derivQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat, vel)
end

function mju_quatIntegrate(quat, vel, scale)
    ccall((:mju_quatIntegrate, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), quat, vel, scale)
end

function mju_quatZ2Vec(quat, vec)
    ccall((:mju_quatZ2Vec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), quat, vec)
end

function mju_mulPose(posres, quatres, pos1, quat1, pos2, quat2)
    ccall((:mju_mulPose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), posres, quatres, pos1, quat1, pos2, quat2)
end

function mju_negPose(posres, quatres, pos, quat)
    ccall((:mju_negPose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), posres, quatres, pos, quat)
end

function mju_trnVecPose(res, pos, quat, vec)
    ccall((:mju_trnVecPose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, pos, quat, vec)
end

function mju_cholFactor(mat, n, mindiag)
    ccall((:mju_cholFactor, libmujuco), Cint, (Ptr{mjtNum}, Cint, mjtNum), mat, n, mindiag)
end

function mju_cholSolve(res, mat, vec, n)
    ccall((:mju_cholSolve, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, mat, vec, n)
end

function mju_cholUpdate(mat, x, n, flg_plus)
    ccall((:mju_cholUpdate, libmujuco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), mat, x, n, flg_plus)
end

function mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)
    ccall((:mju_cholFactorBand, libmujuco), mjtNum, (Ptr{mjtNum}, Cint, Cint, Cint, mjtNum, mjtNum), mat, ntotal, nband, ndense, diagadd, diagmul)
end

function mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)
    ccall((:mju_cholSolveBand, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat, vec, ntotal, nband, ndense)
end

function mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)
    ccall((:mju_band2Dense, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint, mjtByte), res, mat, ntotal, nband, ndense, flg_sym)
end

function mju_dense2Band(res, mat, ntotal, nband, ndense)
    ccall((:mju_dense2Band, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat, ntotal, nband, ndense)
end

function mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nvec, flg_sym)
    ccall((:mju_bandMulMatVec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint, Cint, mjtByte), res, mat, vec, ntotal, nband, ndense, nvec, flg_sym)
end

function mju_bandDiag(i, ntotal, nband, ndense)
    ccall((:mju_bandDiag, libmujuco), Cint, (Cint, Cint, Cint, Cint), i, ntotal, nband, ndense)
end

function mju_eig3(eigval, eigvec, quat, mat)
    ccall((:mju_eig3, libmujuco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), eigval, eigvec, quat, mat)
end

function mju_boxQP(res, R, index, H, g, n, lower, upper)
    ccall((:mju_boxQP, libmujuco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), res, R, index, H, g, n, lower, upper)
end

function mju_boxQPmalloc(res, R, index, H, g, n, lower, upper)
    ccall((:mju_boxQPmalloc, libmujuco), Cvoid, (Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}, Ptr{Ptr{Cint}}, Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}, Cint, Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}), res, R, index, H, g, n, lower, upper)
end

function mju_muscleGain(len, vel, lengthrange, acc0, prm)
    ccall((:mju_muscleGain, libmujuco), mjtNum, (mjtNum, mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}), len, vel, lengthrange, acc0, prm)
end

function mju_muscleBias(len, lengthrange, acc0, prm)
    ccall((:mju_muscleBias, libmujuco), mjtNum, (mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}), len, lengthrange, acc0, prm)
end

function mju_muscleDynamics(ctrl, act, prm)
    ccall((:mju_muscleDynamics, libmujuco), mjtNum, (mjtNum, mjtNum, Ptr{mjtNum}), ctrl, act, prm)
end

function mju_encodePyramid(pyramid, force, mu, dim)
    ccall((:mju_encodePyramid, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), pyramid, force, mu, dim)
end

function mju_decodePyramid(force, pyramid, mu, dim)
    ccall((:mju_decodePyramid, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), force, pyramid, mu, dim)
end

function mju_springDamper(pos0, vel0, Kp, Kv, dt)
    ccall((:mju_springDamper, libmujuco), mjtNum, (mjtNum, mjtNum, mjtNum, mjtNum, mjtNum), pos0, vel0, Kp, Kv, dt)
end

function mju_min(a, b)
    ccall((:mju_min, libmujuco), mjtNum, (mjtNum, mjtNum), a, b)
end

function mju_max(a, b)
    ccall((:mju_max, libmujuco), mjtNum, (mjtNum, mjtNum), a, b)
end

function mju_clip(x, min, max)
    ccall((:mju_clip, libmujuco), mjtNum, (mjtNum, mjtNum, mjtNum), x, min, max)
end

function mju_sign(x)
    ccall((:mju_sign, libmujuco), mjtNum, (mjtNum,), x)
end

function mju_round(x)
    ccall((:mju_round, libmujuco), Cint, (mjtNum,), x)
end

function mju_type2Str(type)
    ccall((:mju_type2Str, libmujuco), Ptr{Cchar}, (Cint,), type)
end

function mju_str2Type(str)
    ccall((:mju_str2Type, libmujuco), Cint, (Ptr{Cchar},), str)
end

function mju_writeNumBytes(nbytes)
    ccall((:mju_writeNumBytes, libmujuco), Ptr{Cchar}, (Csize_t,), nbytes)
end

function mju_warningText(warning, info)
    ccall((:mju_warningText, libmujuco), Ptr{Cchar}, (Cint, Csize_t), warning, info)
end

function mju_isBad(x)
    ccall((:mju_isBad, libmujuco), Cint, (mjtNum,), x)
end

function mju_isZero(vec, n)
    ccall((:mju_isZero, libmujuco), Cint, (Ptr{mjtNum}, Cint), vec, n)
end

function mju_standardNormal(num2)
    ccall((:mju_standardNormal, libmujuco), mjtNum, (Ptr{mjtNum},), num2)
end

function mju_f2n(res, vec, n)
    ccall((:mju_f2n, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{Cfloat}, Cint), res, vec, n)
end

function mju_n2f(res, vec, n)
    ccall((:mju_n2f, libmujuco), Cvoid, (Ptr{Cfloat}, Ptr{mjtNum}, Cint), res, vec, n)
end

function mju_d2n(res, vec, n)
    ccall((:mju_d2n, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{Cdouble}, Cint), res, vec, n)
end

function mju_n2d(res, vec, n)
    ccall((:mju_n2d, libmujuco), Cvoid, (Ptr{Cdouble}, Ptr{mjtNum}, Cint), res, vec, n)
end

function mju_insertionSort(list, n)
    ccall((:mju_insertionSort, libmujuco), Cvoid, (Ptr{mjtNum}, Cint), list, n)
end

function mju_insertionSortInt(list, n)
    ccall((:mju_insertionSortInt, libmujuco), Cvoid, (Ptr{Cint}, Cint), list, n)
end

function mju_Halton(index, base)
    ccall((:mju_Halton, libmujuco), mjtNum, (Cint, Cint), index, base)
end

function mju_strncpy(dst, src, n)
    ccall((:mju_strncpy, libmujuco), Ptr{Cchar}, (Ptr{Cchar}, Ptr{Cchar}, Cint), dst, src, n)
end

function mju_sigmoid(x)
    ccall((:mju_sigmoid, libmujuco), mjtNum, (mjtNum,), x)
end

function mjd_transitionFD(m, d, eps, flg_centered, A, B, C, D)
    ccall((:mjd_transitionFD, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, mjtNum, mjtByte, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, eps, flg_centered, A, B, C, D)
end

function mjd_inverseFD(m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)
    ccall((:mjd_inverseFD, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, mjtNum, mjtByte, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)
end

function mjd_subQuat(qa, qb, Da, Db)
    ccall((:mjd_subQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), qa, qb, Da, Db)
end

function mjd_quatIntegrate(vel, scale, Dquat, Dvel, Dscale)
    ccall((:mjd_quatIntegrate, libmujuco), Cvoid, (Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), vel, scale, Dquat, Dvel, Dscale)
end

function mjp_defaultPlugin(plugin)
    ccall((:mjp_defaultPlugin, libmujuco), Cvoid, (Ptr{mjpPlugin},), plugin)
end

function mjp_registerPlugin(plugin)
    ccall((:mjp_registerPlugin, libmujuco), Cint, (Ptr{mjpPlugin},), plugin)
end

# no prototype is found for this function at mujoco.h:1270:11, please use with caution
function mjp_pluginCount()
    ccall((:mjp_pluginCount, libmujuco), Cint, ())
end

function mjp_getPlugin(name, slot)
    ccall((:mjp_getPlugin, libmujuco), Ptr{mjpPlugin}, (Ptr{Cchar}, Ptr{Cint}), name, slot)
end

function mjp_getPluginAtSlot(slot)
    ccall((:mjp_getPluginAtSlot, libmujuco), Ptr{mjpPlugin}, (Cint,), slot)
end

function mjp_defaultResourceProvider(provider)
    ccall((:mjp_defaultResourceProvider, libmujuco), Cvoid, (Ptr{mjpResourceProvider},), provider)
end

function mjp_registerResourceProvider(provider)
    ccall((:mjp_registerResourceProvider, libmujuco), Cint, (Ptr{mjpResourceProvider},), provider)
end

# no prototype is found for this function at mujoco.h:1287:11, please use with caution
function mjp_resourceProviderCount()
    ccall((:mjp_resourceProviderCount, libmujuco), Cint, ())
end

function mjp_getResourceProvider(resource_name)
    ccall((:mjp_getResourceProvider, libmujuco), Ptr{mjpResourceProvider}, (Ptr{Cchar},), resource_name)
end

function mjp_getResourceProviderAtSlot(slot)
    ccall((:mjp_getResourceProviderAtSlot, libmujuco), Ptr{mjpResourceProvider}, (Cint,), slot)
end

# Skipping MacroDefinition: MUJOCO_HELPER_DLL_IMPORT __attribute__ ( ( visibility ( "default" ) ) )

# Skipping MacroDefinition: MUJOCO_HELPER_DLL_EXPORT __attribute__ ( ( visibility ( "default" ) ) )

# Skipping MacroDefinition: MUJOCO_HELPER_DLL_LOCAL __attribute__ ( ( visibility ( "hidden" ) ) )





const mjPI = 3.141592653589793

const mjMAXVAL = 1.0e10

const mjMINMU = 1.0e-5

const mjMINIMP = 0.0001

const mjMAXIMP = 0.9999

const mjMAXCONPAIR = 50

const mjMAXTREEDEPTH = 50

const mjMAXVFS = 2000

const mjMAXVFSNAME = 1000

const mjNEQDATA = 11

const mjNDYN = 10

const mjNGAIN = 10

const mjNBIAS = 10

const mjNFLUID = 12

const mjNREF = 2

const mjNIMP = 5

const mjNSOLVER = 1000

const mjVFS_PREFIX = "vfs"

# Skipping MacroDefinition: mjPLUGIN_LIB_INIT __attribute__ ( ( constructor ) ) static void _mjplugin_init ( void )

const mjNAUX = 10

const mjMAXTEXTURE = 1000

const mjMINVAL = 1.0e-15

const mjMAXUISECT = 10

const mjMAXUIITEM = 100

const mjMAXUITEXT = 300

const mjMAXUINAME = 40

const mjMAXUIMULTI = 35

const mjMAXUIEDIT = 7

const mjMAXUIRECT = 25

const mjSEPCLOSED = 1000

const mjKEY_ESCAPE = 256

const mjKEY_ENTER = 257

const mjKEY_TAB = 258

const mjKEY_BACKSPACE = 259

const mjKEY_INSERT = 260

const mjKEY_DELETE = 261

const mjKEY_RIGHT = 262

const mjKEY_LEFT = 263

const mjKEY_DOWN = 264

const mjKEY_UP = 265

const mjKEY_PAGE_UP = 266

const mjKEY_PAGE_DOWN = 267

const mjKEY_HOME = 268

const mjKEY_END = 269

const mjKEY_F1 = 290

const mjKEY_F2 = 291

const mjKEY_F3 = 292

const mjKEY_F4 = 293

const mjKEY_F5 = 294

const mjKEY_F6 = 295

const mjKEY_F7 = 296

const mjKEY_F8 = 297

const mjKEY_F9 = 298

const mjKEY_F10 = 299

const mjKEY_F11 = 300

const mjKEY_F12 = 301

const mjKEY_NUMPAD_0 = 320

const mjKEY_NUMPAD_9 = 329

const mjNGROUP = 6

const mjMAXLIGHT = 100

const mjMAXOVERLAY = 500

const mjMAXLINE = 100

const mjMAXLINEPNT = 1000

const mjMAXPLANEGRID = 200















const mjVERSION_HEADER = 237

const mju_sqrt = sqrt

const mju_exp = exp

const mju_sin = sin

const mju_cos = cos

const mju_tan = tan

const mju_asin = asin

const mju_acos = acos

const mju_atan2 = atan

const mju_tanh = tanh

const mju_pow = ^

const mju_abs = abs

const mju_log = log

const mju_log10 = log10

const mju_floor = floor

const mju_ceil = ceil

# exports
const PREFIXES = ["mj"] #["MJ", "mj_"]
for name in names(@__MODULE__; all=true), prefix in PREFIXES
    if startswith(string(name), prefix)
        @eval export $name
    end
end

end # module
