const mjtState = mjtState_
const mjtWarning = mjtWarning_
const mjtTimer = mjtTimer_
"""
	mjContact

# Fields

## dist
distance between nearest points; neg: penetration
## pos
position of contact point: midpoint between geoms
## frame
normal is in [0-2]
## includemargin
include if dist<includemargin=margin-gap
## friction
tangent1, 2, spin, roll1, 2
## solref
constraint solver reference, normal direction
## solreffriction
constraint solver reference, friction directions
## solimp
constraint solver impedance
## mu
friction of regularized cone, set by mj_makeConstraint
## H
cone Hessian, set by mj_updateConstraint
## dim
contact space dimensionality: 1, 3, 4 or 6
## geom1
id of geom 1
## geom2
id of geom 2
## exclude
0: include, 1: in gap, 2: fused, 3: no dofs
## efc_address
address in efc; -1: not included

"""
struct mjContact_
    dist::mjtNum
    pos::NTuple{3,mjtNum}
    frame::NTuple{9,mjtNum}
    includemargin::mjtNum
    friction::NTuple{5,mjtNum}
    solref::NTuple{2,mjtNum}
    solreffriction::NTuple{2,mjtNum}
    solimp::NTuple{5,mjtNum}
    mu::mjtNum
    H::NTuple{36,mjtNum}
    dim::Cint
    geom1::Cint
    geom2::Cint
    exclude::Cint
    efc_address::Cint
end
const mjContact = mjContact_
"""
	mjWarningStat

# Fields

## lastinfo
info from last warning
## number
how many times was warning raised

"""
struct mjWarningStat_
    lastinfo::Cint
    number::Cint
end
const mjWarningStat = mjWarningStat_
"""
	mjTimerStat

# Fields

## duration
cumulative duration
## number
how many times was timer called

"""
struct mjTimerStat_
    duration::mjtNum
    number::Cint
end
const mjTimerStat = mjTimerStat_
"""
	mjSolverStat

# Fields

## improvement
cost reduction, scaled by 1/trace(M(qpos0))
## gradient
gradient norm (primal only, scaled)
## lineslope
slope in linesearch
## nactive
number of active constraints
## nchange
number of constraint state changes
## neval
number of cost evaluations in line search
## nupdate
number of Cholesky updates in line search

"""
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
"""
	mjData

# Fields

## nstack
number of mjtNums that can fit in the arena+stack space
## nbuffer
size of main buffer in bytes
## nplugin
number of plugin instances
## pstack
first available mjtNum address in stack
## parena
first available byte in arena
## maxuse_stack
maximum stack allocation
## maxuse_arena
maximum arena allocation
## maxuse_con
maximum number of contacts
## maxuse_efc
maximum number of scalar constraints
## warning
warning statistics
## timer
timer statistics
## solver
solver statistics per iteration
## solver_iter
number of solver iterations
## solver_nnz
number of non-zeros in Hessian or efc_AR
## solver_fwdinv
forward-inverse comparison: qfrc, efc
## nbodypair_broad
number of body pairs in collision according to the broad-phase
## nbodypair_narrow
number of body pairs actually in collision in the narrow-phase
## ngeompair_mid
number of geom pairs in collision according to the mid-phase
## ngeompair_narrow
number of geom pairs actually in collision in the narrow-phase
## ne
number of equality constraints
## nf
number of friction constraints
## nefc
number of constraints
## nnzJ
number of non-zeros in constraint Jacobian
## ncon
number of detected contacts
## time
simulation time
## energy
potential, kinetic energy
## buffer
main buffer; all pointers point in it                (nbuffer bytes)
## arena
arena+stack buffer                     (nstack*sizeof(mjtNum) bytes)
## qpos
position                                         (nq x 1)
## qvel
velocity                                         (nv x 1)
## act
actuator activation                              (na x 1)
## qacc_warmstart
acceleration used for warmstart                  (nv x 1)
## plugin_state
plugin state                                     (npluginstate x 1)
## ctrl
control                                          (nu x 1)
## qfrc_applied
applied generalized force                        (nv x 1)
## xfrc_applied
applied Cartesian force/torque                   (nbody x 6)
## mocap_pos
positions of mocap bodies                        (nmocap x 3)
## mocap_quat
orientations of mocap bodies                     (nmocap x 4)
## qacc
acceleration                                     (nv x 1)
## act_dot
time-derivative of actuator activation           (na x 1)
## userdata
user data, not touched by engine                 (nuserdata x 1)
## sensordata
sensor data array                                (nsensordata x 1)
## plugin
copy of m->plugin, required for deletion         (nplugin x 1)
## plugin_data
pointer to plugin-managed data structure         (nplugin x 1)
## xpos
Cartesian position of body frame                 (nbody x 3)
## xquat
Cartesian orientation of body frame              (nbody x 4)
## xmat
Cartesian orientation of body frame              (nbody x 9)
## xipos
Cartesian position of body com                   (nbody x 3)
## ximat
Cartesian orientation of body inertia            (nbody x 9)
## xanchor
Cartesian position of joint anchor               (njnt x 3)
## xaxis
Cartesian joint axis                             (njnt x 3)
## geom_xpos
Cartesian geom position                          (ngeom x 3)
## geom_xmat
Cartesian geom orientation                       (ngeom x 9)
## site_xpos
Cartesian site position                          (nsite x 3)
## site_xmat
Cartesian site orientation                       (nsite x 9)
## cam_xpos
Cartesian camera position                        (ncam x 3)
## cam_xmat
Cartesian camera orientation                     (ncam x 9)
## light_xpos
Cartesian light position                         (nlight x 3)
## light_xdir
Cartesian light direction                        (nlight x 3)
## subtree_com
center of mass of each subtree                   (nbody x 3)
## cdof
com-based motion axis of each dof                (nv x 6)
## cinert
com-based body inertia and mass                  (nbody x 10)
## ten_wrapadr
start address of tendon's path                   (ntendon x 1)
## ten_wrapnum
number of wrap points in path                    (ntendon x 1)
## ten_J_rownnz
number of non-zeros in Jacobian row              (ntendon x 1)
## ten_J_rowadr
row start address in colind array                (ntendon x 1)
## ten_J_colind
column indices in sparse Jacobian                (ntendon x nv)
## ten_length
tendon lengths                                   (ntendon x 1)
## ten_J
tendon Jacobian                                  (ntendon x nv)
## wrap_obj
geom id; -1: site; -2: pulley                    (nwrap*2 x 1)
## wrap_xpos
Cartesian 3D points in all path                  (nwrap*2 x 3)
## actuator_length
actuator lengths                                 (nu x 1)
## actuator_moment
actuator moments                                 (nu x nv)
## crb
com-based composite inertia and mass             (nbody x 10)
## qM
total inertia (sparse)                           (nM x 1)
## qLD
L'*D*L factorization of M (sparse)               (nM x 1)
## qLDiagInv
1/diag(D)                                        (nv x 1)
## qLDiagSqrtInv
1/sqrt(diag(D))                                  (nv x 1)
## bvh_active
volume has been added to collisions              (nbvh x 1)
## ten_velocity
tendon velocities                                (ntendon x 1)
## actuator_velocity
actuator velocities                              (nu x 1)
## cvel
com-based velocity [3D rot; 3D tran]             (nbody x 6)
## cdof_dot
time-derivative of cdof                          (nv x 6)
## qfrc_bias
C(qpos,qvel)                                     (nv x 1)
## qfrc_passive
passive force                                    (nv x 1)
## efc_vel
velocity in constraint space: J*qvel             (nefc x 1)
## efc_aref
reference pseudo-acceleration                    (nefc x 1)
## subtree_linvel
linear velocity of subtree com                   (nbody x 3)
## subtree_angmom
angular momentum about subtree com               (nbody x 3)
## qH
L'*D*L factorization of modified M               (nM x 1)
## qHDiagInv
1/diag(D) of modified M                          (nv x 1)
## D_rownnz
non-zeros in each row                            (nv x 1)
## D_rowadr
address of each row in D_colind                  (nv x 1)
## D_colind
column indices of non-zeros                      (nD x 1)
## B_rownnz
non-zeros in each row                            (nbody x 1)
## B_rowadr
address of each row in B_colind                  (nbody x 1)
## B_colind
column indices of non-zeros                      (nB x 1)
## qDeriv
d (passive + actuator - bias) / d qvel           (nD x 1)
## qLU
sparse LU of (qM - dt*qDeriv)                    (nD x 1)
## actuator_force
actuator force in actuation space                (nu x 1)
## qfrc_actuator
actuator force                                   (nv x 1)
## qfrc_smooth
net unconstrained force                          (nv x 1)
## qacc_smooth
unconstrained acceleration                       (nv x 1)
## qfrc_constraint
constraint force                                 (nv x 1)
## qfrc_inverse
net external force; should equal:                (nv x 1)qfrc_applied + J'*xfrc_applied + qfrc_actuator
## cacc
com-based acceleration                           (nbody x 6)
## cfrc_int
com-based interaction force with parent          (nbody x 6)
## cfrc_ext
com-based external force on body                 (nbody x 6)
## contact
list of all detected contacts                    (ncon x 1)
## efc_type
constraint type (mjtConstraint)                  (nefc x 1)
## efc_id
id of object of specified type                   (nefc x 1)
## efc_J_rownnz
number of non-zeros in constraint Jacobian row   (nefc x 1)
## efc_J_rowadr
row start address in colind array                (nefc x 1)
## efc_J_rowsuper
number of subsequent rows in supernode           (nefc x 1)
## efc_J_colind
column indices in constraint Jacobian            (nnzJ x 1)
## efc_JT_rownnz
number of non-zeros in constraint Jacobian row T (nv x 1)
## efc_JT_rowadr
row start address in colind array              T (nv x 1)
## efc_JT_rowsuper
number of subsequent rows in supernode         T (nv x 1)
## efc_JT_colind
column indices in constraint Jacobian          T (nnzJ x 1)
## efc_J
constraint Jacobian                              (nnzJ x 1)
## efc_JT
constraint Jacobian transposed                   (nnzJ x 1)
## efc_pos
constraint position (equality, contact)          (nefc x 1)
## efc_margin
inclusion margin (contact)                       (nefc x 1)
## efc_frictionloss
frictionloss (friction)                          (nefc x 1)
## efc_diagApprox
approximation to diagonal of A                   (nefc x 1)
## efc_KBIP
stiffness, damping, impedance, imp'              (nefc x 4)
## efc_D
constraint mass                                  (nefc x 1)
## efc_R
inverse constraint mass                          (nefc x 1)
## efc_b
linear cost term: J*qacc_smooth - aref            (nefc x 1)
## efc_force
constraint force in constraint space              (nefc x 1)
## efc_state
constraint state (mjtConstraintState)             (nefc x 1)
## efc_AR_rownnz
number of non-zeros in AR                         (nefc x 1)
## efc_AR_rowadr
row start address in colind array                 (nefc x 1)
## efc_AR_colind
column indices in sparse AR                       (nefc x nefc)
## efc_AR
J*inv(M)*J' + R                                   (nefc x nefc)

"""
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
    warning::NTuple{8,mjWarningStat}
    timer::NTuple{13,mjTimerStat}
    solver::NTuple{1000,mjSolverStat}
    solver_iter::Cint
    solver_nnz::Cint
    solver_fwdinv::NTuple{2,mjtNum}
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
    energy::NTuple{2,mjtNum}
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
const mjtDisableBit = mjtDisableBit_
const mjtEnableBit = mjtEnableBit_
const mjtJoint = mjtJoint_
const mjtGeom = mjtGeom_
const mjtCamLight = mjtCamLight_
const mjtTexture = mjtTexture_
const mjtIntegrator = mjtIntegrator_
const mjtCollision = mjtCollision_
const mjtCone = mjtCone_
const mjtJacobian = mjtJacobian_
const mjtSolver = mjtSolver_
const mjtEq = mjtEq_
const mjtWrap = mjtWrap_
const mjtTrn = mjtTrn_
const mjtDyn = mjtDyn_
const mjtGain = mjtGain_
const mjtBias = mjtBias_
const mjtObj = mjtObj_
const mjtConstraint = mjtConstraint_
const mjtConstraintState = mjtConstraintState_
const mjtSensor = mjtSensor_
const mjtStage = mjtStage_
const mjtDataType = mjtDataType_
const mjtLRMode = mjtLRMode_
"""
	mjLROpt

# Fields

## mode
which actuators to process (mjtLRMode)
## useexisting
use existing length range if available
## uselimit
use joint and tendon limits if available
## accel
target acceleration used to compute force
## maxforce
maximum force; 0: no limit
## timeconst
time constant for velocity reduction; min 0.01
## timestep
simulation timestep; 0: use mjOption.timestep
## inttotal
total simulation time interval
## interval
evaluation time interval (at the end)
## tolrange
convergence tolerance (relative to range)

"""
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
"""
	mjVFS

# Fields

## nfile
number of files present
## filename
file name without path
## filesize
file size in bytes
## filedata
buffer with file data

"""
struct mjVFS_
    nfile::Cint
    filename::NTuple{2000,NTuple{1000,Cchar}}
    filesize::NTuple{2000,Cint}
    filedata::NTuple{2000,Ptr{Cvoid}}
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
"""
	mjOption

# Fields

## timestep
timestep
## apirate
update rate for remote API (Hz)
## impratio
ratio of friction-to-normal contact impedance
## tolerance
main solver tolerance
## noslip_tolerance
noslip solver tolerance
## mpr_tolerance
MPR solver tolerance
## gravity
gravitational acceleration
## wind
wind (for lift, drag and viscosity)
## magnetic
global magnetic flux
## density
density of medium
## viscosity
viscosity of medium
## o_margin
margin
## o_solref
solref
## o_solimp
solimp
## integrator
integration mode (mjtIntegrator)
## collision
collision mode (mjtCollision)
## cone
type of friction cone (mjtCone)
## jacobian
type of Jacobian (mjtJacobian)
## solver
solver algorithm (mjtSolver)
## iterations
maximum number of main solver iterations
## noslip_iterations
maximum number of noslip solver iterations
## mpr_iterations
maximum number of MPR solver iterations
## disableflags
bit flags for disabling standard features
## enableflags
bit flags for enabling optional features

"""
struct mjOption_
    timestep::mjtNum
    apirate::mjtNum
    impratio::mjtNum
    tolerance::mjtNum
    noslip_tolerance::mjtNum
    mpr_tolerance::mjtNum
    gravity::NTuple{3,mjtNum}
    wind::NTuple{3,mjtNum}
    magnetic::NTuple{3,mjtNum}
    density::mjtNum
    viscosity::mjtNum
    o_margin::mjtNum
    o_solref::NTuple{2,mjtNum}
    o_solimp::NTuple{5,mjtNum}
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
struct var"##Ctag#304"
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
struct var"##Ctag#305"
    shadowsize::Cint
    offsamples::Cint
    numslices::Cint
    numstacks::Cint
    numquads::Cint
end
struct var"##Ctag#306"
    ambient::NTuple{3,Cfloat}
    diffuse::NTuple{3,Cfloat}
    specular::NTuple{3,Cfloat}
    active::Cint
end
struct var"##Ctag#307"
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
struct var"##Ctag#308"
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
struct var"##Ctag#309"
    fog::NTuple{4,Cfloat}
    haze::NTuple{4,Cfloat}
    force::NTuple{4,Cfloat}
    inertia::NTuple{4,Cfloat}
    joint::NTuple{4,Cfloat}
    actuator::NTuple{4,Cfloat}
    actuatornegative::NTuple{4,Cfloat}
    actuatorpositive::NTuple{4,Cfloat}
    com::NTuple{4,Cfloat}
    camera::NTuple{4,Cfloat}
    light::NTuple{4,Cfloat}
    selectpoint::NTuple{4,Cfloat}
    connect::NTuple{4,Cfloat}
    contactpoint::NTuple{4,Cfloat}
    contactforce::NTuple{4,Cfloat}
    contactfriction::NTuple{4,Cfloat}
    contacttorque::NTuple{4,Cfloat}
    contactgap::NTuple{4,Cfloat}
    rangefinder::NTuple{4,Cfloat}
    constraint::NTuple{4,Cfloat}
    slidercrank::NTuple{4,Cfloat}
    crankbroken::NTuple{4,Cfloat}
end
"""
	mjVisual
"""
struct mjVisual_
    data::NTuple{568,UInt8}
end
const mjVisual = mjVisual_
"""
	mjStatistic

# Fields

## meaninertia
mean diagonal inertia
## meanmass
mean body mass
## meansize
mean body size
## extent
spatial extent
## center
center of model

"""
struct mjStatistic_
    meaninertia::mjtNum
    meanmass::mjtNum
    meansize::mjtNum
    extent::mjtNum
    center::NTuple{3,mjtNum}
end
const mjStatistic = mjStatistic_
"""
	mjModel

# Fields

## nq
number of generalized coordinates = dim(qpos)
## nv
number of degrees of freedom = dim(qvel)
## nu
number of actuators/controls = dim(ctrl)
## na
number of activation states = dim(act)
## nbody
number of bodies
## nbvh
number of total bounding volumes in all bodies
## njnt
number of joints
## ngeom
number of geoms
## nsite
number of sites
## ncam
number of cameras
## nlight
number of lights
## nmesh
number of meshes
## nmeshvert
number of vertices in all meshes
## nmeshnormal
number of normals in all meshes
## nmeshtexcoord
number of texcoords in all meshes
## nmeshface
number of triangular faces in all meshes
## nmeshgraph
number of ints in mesh auxiliary data
## nskin
number of skins
## nskinvert
number of vertices in all skins
## nskintexvert
number of vertiex with texcoords in all skins
## nskinface
number of triangular faces in all skins
## nskinbone
number of bones in all skins
## nskinbonevert
number of vertices in all skin bones
## nhfield
number of heightfields
## nhfielddata
number of data points in all heightfields
## ntex
number of textures
## ntexdata
number of bytes in texture rgb data
## nmat
number of materials
## npair
number of predefined geom pairs
## nexclude
number of excluded geom pairs
## neq
number of equality constraints
## ntendon
number of tendons
## nwrap
number of wrap objects in all tendon paths
## nsensor
number of sensors
## nnumeric
number of numeric custom fields
## nnumericdata
number of mjtNums in all numeric fields
## ntext
number of text custom fields
## ntextdata
number of mjtBytes in all text fields
## ntuple
number of tuple custom fields
## ntupledata
number of objects in all tuple fields
## nkey
number of keyframes
## nmocap
number of mocap bodies
## nplugin
number of plugin instances
## npluginattr
number of chars in all plugin config attributes
## nuser_body
number of mjtNums in body_user
## nuser_jnt
number of mjtNums in jnt_user
## nuser_geom
number of mjtNums in geom_user
## nuser_site
number of mjtNums in site_user
## nuser_cam
number of mjtNums in cam_user
## nuser_tendon
number of mjtNums in tendon_user
## nuser_actuator
number of mjtNums in actuator_user
## nuser_sensor
number of mjtNums in sensor_user
## nnames
number of chars in all names
## nnames_map
number of slots in the names hash map
## nM
number of non-zeros in sparse inertia matrix
## nD
number of non-zeros in sparse dof-dof matrix
## nB
number of non-zeros in sparse body-dof matrix
## nemax
number of potential equality-constraint rows
## njmax
number of available rows in constraint Jacobian
## nconmax
number of potential contacts in contact list
## nstack
number of fields in mjData stack
## nuserdata
number of extra fields in mjData
## nsensordata
number of fields in sensor data vector
## npluginstate
number of fields in the plugin state vector
## nbuffer
number of bytes in buffer
## opt
physics options
## vis
visualization options
## stat
model statistics
## buffer
main buffer; all pointers point in it    (nbuffer)
## qpos0
qpos values at default pose              (nq x 1)
## qpos_spring
reference pose for springs               (nq x 1)
## body_parentid
id of body's parent                      (nbody x 1)
## body_rootid
id of root above body                    (nbody x 1)
## body_weldid
id of body that this body is welded to   (nbody x 1)
## body_mocapid
id of mocap data; -1: none               (nbody x 1)
## body_jntnum
number of joints for this body           (nbody x 1)
## body_jntadr
start addr of joints; -1: no joints      (nbody x 1)
## body_dofnum
number of motion degrees of freedom      (nbody x 1)
## body_dofadr
start addr of dofs; -1: no dofs          (nbody x 1)
## body_geomnum
number of geoms                          (nbody x 1)
## body_geomadr
start addr of geoms; -1: no geoms        (nbody x 1)
## body_simple
body is simple (has diagonal M)          (nbody x 1)
## body_sameframe
inertial frame is same as body frame     (nbody x 1)
## body_pos
position offset rel. to parent body      (nbody x 3)
## body_quat
orientation offset rel. to parent body   (nbody x 4)
## body_ipos
local position of center of mass         (nbody x 3)
## body_iquat
local orientation of inertia ellipsoid   (nbody x 4)
## body_mass
mass                                     (nbody x 1)
## body_subtreemass
mass of subtree starting at this body    (nbody x 1)
## body_inertia
diagonal inertia in ipos/iquat frame     (nbody x 3)
## body_invweight0
mean inv inert in qpos0 (trn, rot)       (nbody x 2)
## body_gravcomp
antigravity force, units of body weight  (nbody x 1)
## body_user
user data                                (nbody x nuser_body)
## body_plugin
plugin instance id; -1: not in use       (nbody x 1)
## body_bvhadr
address of bvh root                      (nbody x 1)
## body_bvhnum
number of bounding volumes               (nbody x 1)
## bvh_depth
depth in the bounding volume hierarchy   (nbvh x 1)
## bvh_child
left and right children in tree          (nbvh x 2)
## bvh_geomid
geom id of the node; -1: non-leaf        (nbvh x 1)
## bvh_aabb
bounding box of node (center, size)      (nbvh x 6)
## jnt_type
type of joint (mjtJoint)                 (njnt x 1)
## jnt_qposadr
start addr in 'qpos' for joint's data    (njnt x 1)
## jnt_dofadr
start addr in 'qvel' for joint's data    (njnt x 1)
## jnt_bodyid
id of joint's body                       (njnt x 1)
## jnt_group
group for visibility                     (njnt x 1)
## jnt_limited
does joint have limits                   (njnt x 1)
## jnt_actfrclimited
does joint have actuator force limits    (njnt x 1)
## jnt_solref
constraint solver reference: limit       (njnt x mjNREF)
## jnt_solimp
constraint solver impedance: limit       (njnt x mjNIMP)
## jnt_pos
local anchor position                    (njnt x 3)
## jnt_axis
local joint axis                         (njnt x 3)
## jnt_stiffness
stiffness coefficient                    (njnt x 1)
## jnt_range
joint limits                             (njnt x 2)
## jnt_actfrcrange
range of total actuator force            (njnt x 2)
## jnt_margin
min distance for limit detection         (njnt x 1)
## jnt_user
user data                                (njnt x nuser_jnt)
## dof_bodyid
id of dof's body                         (nv x 1)
## dof_jntid
id of dof's joint                        (nv x 1)
## dof_parentid
id of dof's parent; -1: none             (nv x 1)
## dof_Madr
dof address in M-diagonal                (nv x 1)
## dof_simplenum
number of consecutive simple dofs        (nv x 1)
## dof_solref
constraint solver reference:frictionloss (nv x mjNREF)
## dof_solimp
constraint solver impedance:frictionloss (nv x mjNIMP)
## dof_frictionloss
dof friction loss                        (nv x 1)
## dof_armature
dof armature inertia/mass                (nv x 1)
## dof_damping
damping coefficient                      (nv x 1)
## dof_invweight0
diag. inverse inertia in qpos0           (nv x 1)
## dof_M0
diag. inertia in qpos0                   (nv x 1)
## geom_type
geometric type (mjtGeom)                 (ngeom x 1)
## geom_contype
geom contact type                        (ngeom x 1)
## geom_conaffinity
geom contact affinity                    (ngeom x 1)
## geom_condim
contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
## geom_bodyid
id of geom's body                        (ngeom x 1)
## geom_dataid
id of geom's mesh/hfield; -1: none       (ngeom x 1)
## geom_matid
material id for rendering; -1: none      (ngeom x 1)
## geom_group
group for visibility                     (ngeom x 1)
## geom_priority
geom contact priority                    (ngeom x 1)
## geom_sameframe
same as body frame (1) or iframe (2)     (ngeom x 1)
## geom_solmix
mixing coef for solref/imp in geom pair  (ngeom x 1)
## geom_solref
constraint solver reference: contact     (ngeom x mjNREF)
## geom_solimp
constraint solver impedance: contact     (ngeom x mjNIMP)
## geom_size
geom-specific size parameters            (ngeom x 3)
## geom_aabb
bounding box, (center, size)             (ngeom x 6)
## geom_rbound
radius of bounding sphere                (ngeom x 1)
## geom_pos
local position offset rel. to body       (ngeom x 3)
## geom_quat
local orientation offset rel. to body    (ngeom x 4)
## geom_friction
friction for (slide, spin, roll)         (ngeom x 3)
## geom_margin
detect contact if dist<margin(ngeom x 1)
## geom_gap
include in solver if dist<margin-gap     (ngeom x 1)
## geom_fluid
fluid interaction parameters             (ngeom x mjNFLUID)
## geom_user
user data                                (ngeom x nuser_geom)
## geom_rgba
rgba when material is omitted            (ngeom x 4)
## site_type
geom type for rendering (mjtGeom)        (nsite x 1)
## site_bodyid
id of site's body                        (nsite x 1)
## site_matid
material id for rendering; -1: none      (nsite x 1)
## site_group
group for visibility                     (nsite x 1)
## site_sameframe
same as body frame (1) or iframe (2)     (nsite x 1)
## site_size
geom size for rendering                  (nsite x 3)
## site_pos
local position offset rel. to body       (nsite x 3)
## site_quat
local orientation offset rel. to body    (nsite x 4)
## site_user
user data                                (nsite x nuser_site)
## site_rgba
rgba when material is omitted            (nsite x 4)
## cam_mode
camera tracking mode (mjtCamLight)       (ncam x 1)
## cam_bodyid
id of camera's body                      (ncam x 1)
## cam_targetbodyid
id of targeted body; -1: none            (ncam x 1)
## cam_pos
position rel. to body frame              (ncam x 3)
## cam_quat
orientation rel. to body frame           (ncam x 4)
## cam_poscom0
global position rel. to sub-com in qpos0 (ncam x 3)
## cam_pos0
global position rel. to body in qpos0    (ncam x 3)
## cam_mat0
global orientation in qpos0              (ncam x 9)
## cam_fovy
y-field of view (deg)                    (ncam x 1)
## cam_ipd
inter-pupilary distance                  (ncam x 1)
## cam_user
user data                                (ncam x nuser_cam)
## light_mode
light tracking mode (mjtCamLight)        (nlight x 1)
## light_bodyid
id of light's body                       (nlight x 1)
## light_targetbodyid
id of targeted body; -1: none            (nlight x 1)
## light_directional
directional light                        (nlight x 1)
## light_castshadow
does light cast shadows                  (nlight x 1)
## light_active
is light on                              (nlight x 1)
## light_pos
position rel. to body frame              (nlight x 3)
## light_dir
direction rel. to body frame             (nlight x 3)
## light_poscom0
global position rel. to sub-com in qpos0 (nlight x 3)
## light_pos0
global position rel. to body in qpos0    (nlight x 3)
## light_dir0
global direction in qpos0                (nlight x 3)
## light_attenuation
OpenGL attenuation (quadratic model)     (nlight x 3)
## light_cutoff
OpenGL cutoff                            (nlight x 1)
## light_exponent
OpenGL exponent                          (nlight x 1)
## light_ambient
ambient rgb (alpha=1)                    (nlight x 3)
## light_diffuse
diffuse rgb (alpha=1)                    (nlight x 3)
## light_specular
specular rgb (alpha=1)                   (nlight x 3)
## mesh_vertadr
first vertex address                     (nmesh x 1)
## mesh_vertnum
number of vertices                       (nmesh x 1)
## mesh_faceadr
first face address                       (nmesh x 1)
## mesh_facenum
number of faces                          (nmesh x 1)
## mesh_bvhadr
address of bvh root                      (nmesh x 1)
## mesh_bvhnum
number of bvh                            (nmesh x 1)
## mesh_normaladr
first normal address                     (nmesh x 1)
## mesh_normalnum
number of normals                        (nmesh x 1)
## mesh_texcoordadr
texcoord data address; -1: no texcoord   (nmesh x 1)
## mesh_texcoordnum
number of texcoord                       (nmesh x 1)
## mesh_graphadr
graph data address; -1: no graph         (nmesh x 1)
## mesh_vert
vertex positions for all meshes          (nmeshvert x 3)
## mesh_normal
normals for all meshes                   (nmeshnormal x 3)
## mesh_texcoord
vertex texcoords for all meshes          (nmeshtexcoord x 2)
## mesh_face
vertex face data                         (nmeshface x 3)
## mesh_facenormal
normal face data                         (nmeshface x 3)
## mesh_facetexcoord
texture face data                        (nmeshface x 3)
## mesh_graph
convex graph data                        (nmeshgraph x 1)
## skin_matid
skin material id; -1: none               (nskin x 1)
## skin_group
group for visibility                     (nskin x 1)
## skin_rgba
skin rgba                                (nskin x 4)
## skin_inflate
inflate skin in normal direction         (nskin x 1)
## skin_vertadr
first vertex address                     (nskin x 1)
## skin_vertnum
number of vertices                       (nskin x 1)
## skin_texcoordadr
texcoord data address; -1: no texcoord   (nskin x 1)
## skin_faceadr
first face address                       (nskin x 1)
## skin_facenum
number of faces                          (nskin x 1)
## skin_boneadr
first bone in skin                       (nskin x 1)
## skin_bonenum
number of bones in skin                  (nskin x 1)
## skin_vert
vertex positions for all skin meshes     (nskinvert x 3)
## skin_texcoord
vertex texcoords for all skin meshes     (nskintexvert x 2)
## skin_face
triangle faces for all skin meshes       (nskinface x 3)
## skin_bonevertadr
first vertex in each bone                (nskinbone x 1)
## skin_bonevertnum
number of vertices in each bone          (nskinbone x 1)
## skin_bonebindpos
bind pos of each bone                    (nskinbone x 3)
## skin_bonebindquat
bind quat of each bone                   (nskinbone x 4)
## skin_bonebodyid
body id of each bone                     (nskinbone x 1)
## skin_bonevertid
mesh ids of vertices in each bone        (nskinbonevert x 1)
## skin_bonevertweight
weights of vertices in each bone         (nskinbonevert x 1)
## hfield_size
(x, y, z_top, z_bottom)                  (nhfield x 4)
## hfield_nrow
number of rows in grid                   (nhfield x 1)
## hfield_ncol
number of columns in grid                (nhfield x 1)
## hfield_adr
address in hfield_data                   (nhfield x 1)
## hfield_data
elevation data                           (nhfielddata x 1)
## tex_type
texture type (mjtTexture)                (ntex x 1)
## tex_height
number of rows in texture image          (ntex x 1)
## tex_width
number of columns in texture image       (ntex x 1)
## tex_adr
address in rgb                           (ntex x 1)
## tex_rgb
rgb (alpha = 1)                          (ntexdata x 1)
## mat_texid
texture id; -1: none                     (nmat x 1)
## mat_texuniform
make texture cube uniform                (nmat x 1)
## mat_texrepeat
texture repetition for 2d mapping        (nmat x 2)
## mat_emission
emission (x rgb)                         (nmat x 1)
## mat_specular
specular (x white)                       (nmat x 1)
## mat_shininess
shininess coef                           (nmat x 1)
## mat_reflectance
reflectance (0: disable)                 (nmat x 1)
## mat_rgba
rgba                                     (nmat x 4)
## pair_dim
contact dimensionality                   (npair x 1)
## pair_geom1
id of geom1                              (npair x 1)
## pair_geom2
id of geom2                              (npair x 1)
## pair_signature
(body1+1)<<16 + body2+1                (npair x 1)
## pair_solref
solver reference: contact normal         (npair x mjNREF)
## pair_solreffriction
solver reference: contact friction       (npair x mjNREF)
## pair_solimp
solver impedance: contact                (npair x mjNIMP)
## pair_margin
detect contact if dist<margin(npair x 1)
## pair_gap
include in solver if dist<margin-gap     (npair x 1)
## pair_friction
tangent1, 2, spin, roll1, 2              (npair x 5)
## exclude_signature
(body1+1)<<16 + body2+1                (nexclude x 1)
## eq_type
constraint type (mjtEq)                  (neq x 1)
## eq_obj1id
id of object 1                           (neq x 1)
## eq_obj2id
id of object 2                           (neq x 1)
## eq_active
enable/disable constraint                (neq x 1)
## eq_solref
constraint solver reference              (neq x mjNREF)
## eq_solimp
constraint solver impedance              (neq x mjNIMP)
## eq_data
numeric data for constraint              (neq x mjNEQDATA)
## tendon_adr
address of first object in tendon's path (ntendon x 1)
## tendon_num
number of objects in tendon's path       (ntendon x 1)
## tendon_matid
material id for rendering                (ntendon x 1)
## tendon_group
group for visibility                     (ntendon x 1)
## tendon_limited
does tendon have length limits           (ntendon x 1)
## tendon_width
width for rendering                      (ntendon x 1)
## tendon_solref_lim
constraint solver reference: limit       (ntendon x mjNREF)
## tendon_solimp_lim
constraint solver impedance: limit       (ntendon x mjNIMP)
## tendon_solref_fri
constraint solver reference: friction    (ntendon x mjNREF)
## tendon_solimp_fri
constraint solver impedance: friction    (ntendon x mjNIMP)
## tendon_range
tendon length limits                     (ntendon x 2)
## tendon_margin
min distance for limit detection         (ntendon x 1)
## tendon_stiffness
stiffness coefficient                    (ntendon x 1)
## tendon_damping
damping coefficient                      (ntendon x 1)
## tendon_frictionloss
loss due to friction                     (ntendon x 1)
## tendon_lengthspring
spring resting length range              (ntendon x 2)
## tendon_length0
tendon length in qpos0                   (ntendon x 1)
## tendon_invweight0
inv. weight in qpos0                     (ntendon x 1)
## tendon_user
user data                                (ntendon x nuser_tendon)
## tendon_rgba
rgba when material is omitted            (ntendon x 4)
## wrap_type
wrap object type (mjtWrap)               (nwrap x 1)
## wrap_objid
object id: geom, site, joint             (nwrap x 1)
## wrap_prm
divisor, joint coef, or site id          (nwrap x 1)
## actuator_trntype
transmission type (mjtTrn)               (nu x 1)
## actuator_dyntype
dynamics type (mjtDyn)                   (nu x 1)
## actuator_gaintype
gain type (mjtGain)                      (nu x 1)
## actuator_biastype
bias type (mjtBias)                      (nu x 1)
## actuator_trnid
transmission id: joint, tendon, site     (nu x 2)
## actuator_actadr
first activation address; -1: stateless  (nu x 1)
## actuator_actnum
number of activation variables           (nu x 1)
## actuator_group
group for visibility                     (nu x 1)
## actuator_ctrllimited
is control limited                       (nu x 1)
## actuator_forcelimited
is force limited                         (nu x 1)
## actuator_actlimited
is activation limited                    (nu x 1)
## actuator_dynprm
dynamics parameters                      (nu x mjNDYN)
## actuator_gainprm
gain parameters                          (nu x mjNGAIN)
## actuator_biasprm
bias parameters                          (nu x mjNBIAS)
## actuator_ctrlrange
range of controls                        (nu x 2)
## actuator_forcerange
range of forces                          (nu x 2)
## actuator_actrange
range of activations                     (nu x 2)
## actuator_gear
scale length and transmitted force       (nu x 6)
## actuator_cranklength
crank length for slider-crank            (nu x 1)
## actuator_acc0
acceleration from unit force in qpos0    (nu x 1)
## actuator_length0
actuator length in qpos0                 (nu x 1)
## actuator_lengthrange
feasible actuator length range           (nu x 2)
## actuator_user
user data                                (nu x nuser_actuator)
## actuator_plugin
plugin instance id; -1: not a plugin     (nu x 1)
## sensor_type
sensor type (mjtSensor)                  (nsensor x 1)
## sensor_datatype
numeric data type (mjtDataType)          (nsensor x 1)
## sensor_needstage
required compute stage (mjtStage)        (nsensor x 1)
## sensor_objtype
type of sensorized object (mjtObj)       (nsensor x 1)
## sensor_objid
id of sensorized object                  (nsensor x 1)
## sensor_reftype
type of reference frame (mjtObj)         (nsensor x 1)
## sensor_refid
id of reference frame; -1: global frame  (nsensor x 1)
## sensor_dim
number of scalar outputs                 (nsensor x 1)
## sensor_adr
address in sensor array                  (nsensor x 1)
## sensor_cutoff
cutoff for real and positive; 0: ignore  (nsensor x 1)
## sensor_noise
noise standard deviation                 (nsensor x 1)
## sensor_user
user data                                (nsensor x nuser_sensor)
## sensor_plugin
plugin instance id; -1: not a plugin     (nsensor x 1)
## plugin
globally registered plugin slot number   (nplugin x 1)
## plugin_stateadr
address in the plugin state array        (nplugin x 1)
## plugin_statenum
number of states in the plugin instance  (nplugin x 1)
## plugin_attr
config attributes of plugin instances    (npluginattr x 1)
## plugin_attradr
address to each instance's config attrib (nplugin x 1)
## numeric_adr
address of field in numeric_data         (nnumeric x 1)
## numeric_size
size of numeric field                    (nnumeric x 1)
## numeric_data
array of all numeric fields              (nnumericdata x 1)
## text_adr
address of text in text_data             (ntext x 1)
## text_size
size of text field (strlen+1)            (ntext x 1)
## text_data
array of all text fields (0-terminated)  (ntextdata x 1)
## tuple_adr
address of text in text_data             (ntuple x 1)
## tuple_size
number of objects in tuple               (ntuple x 1)
## tuple_objtype
array of object types in all tuples      (ntupledata x 1)
## tuple_objid
array of object ids in all tuples        (ntupledata x 1)
## tuple_objprm
array of object params in all tuples     (ntupledata x 1)
## key_time
key time                                 (nkey x 1)
## key_qpos
key position                             (nkey x nq)
## key_qvel
key velocity                             (nkey x nv)
## key_act
key activation                           (nkey x na)
## key_mpos
key mocap position                       (nkey x 3*nmocap)
## key_mquat
key mocap quaternion                     (nkey x 4*nmocap)
## key_ctrl
key control                              (nkey x nu)
## name_bodyadr
body name pointers                       (nbody x 1)
## name_jntadr
joint name pointers                      (njnt x 1)
## name_geomadr
geom name pointers                       (ngeom x 1)
## name_siteadr
site name pointers                       (nsite x 1)
## name_camadr
camera name pointers                     (ncam x 1)
## name_lightadr
light name pointers                      (nlight x 1)
## name_meshadr
mesh name pointers                       (nmesh x 1)
## name_skinadr
skin name pointers                       (nskin x 1)
## name_hfieldadr
hfield name pointers                     (nhfield x 1)
## name_texadr
texture name pointers                    (ntex x 1)
## name_matadr
material name pointers                   (nmat x 1)
## name_pairadr
geom pair name pointers                  (npair x 1)
## name_excludeadr
exclude name pointers                    (nexclude x 1)
## name_eqadr
equality constraint name pointers        (neq x 1)
## name_tendonadr
tendon name pointers                     (ntendon x 1)
## name_actuatoradr
actuator name pointers                   (nu x 1)
## name_sensoradr
sensor name pointers                     (nsensor x 1)
## name_numericadr
numeric name pointers                    (nnumeric x 1)
## name_textadr
text name pointers                       (ntext x 1)
## name_tupleadr
tuple name pointers                      (ntuple x 1)
## name_keyadr
keyframe name pointers                   (nkey x 1)
## name_pluginadr
plugin instance name pointers            (nplugin x 1)
## names
names of all objects, 0-terminated       (nnames x 1)
## names_map
internal hash map of names               (nnames_map x 1)

"""
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
struct mjpResourceProvider_
    prefix::Ptr{Cchar}
    open::mjfOpenResource
    read::mjfReadResource
    close::mjfCloseResource
    getdir::mjfGetResourceDir
    data::Ptr{Cvoid}
end
const mjpResourceProvider = mjpResourceProvider_
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
const mjtGridPos = mjtGridPos_
const mjtFramebuffer = mjtFramebuffer_
const mjtFontScale = mjtFontScale_
const mjtFont = mjtFont_
"""
	mjrRect

# Fields

## left
left (usually 0)
## bottom
bottom (usually 0)
## width
width (usually buffer width)
## height
height (usually buffer height)

"""
struct mjrRect_
    left::Cint
    bottom::Cint
    width::Cint
    height::Cint
end
const mjrRect = mjrRect_
"""
	mjrContext

# Fields

## lineWidth
line width for wireframe rendering
## shadowClip
clipping radius for directional lights
## shadowScale
fraction of light cutoff for spot lights
## fogStart
fog start = stat.extent * vis.map.fogstart
## fogEnd
fog end = stat.extent * vis.map.fogend
## fogRGBA
fog rgba
## shadowSize
size of shadow map texture
## offWidth
width of offscreen buffer
## offHeight
height of offscreen buffer
## offSamples
number of offscreen buffer multisamples
## fontScale
font scale
## auxWidth
auxiliary buffer width
## auxHeight
auxiliary buffer height
## auxSamples
auxiliary buffer multisamples
## offFBO
offscreen framebuffer object
## offFBO_r
offscreen framebuffer for resolving multisamples
## offColor
offscreen color buffer
## offColor_r
offscreen color buffer for resolving multisamples
## offDepthStencil
offscreen depth and stencil buffer
## offDepthStencil_r
offscreen depth and stencil buffer for resolving multisamples
## shadowFBO
shadow map framebuffer object
## shadowTex
shadow map texture
## auxFBO
auxiliary framebuffer object
## auxFBO_r
auxiliary framebuffer object for resolving
## auxColor
auxiliary color buffer
## auxColor_r
auxiliary color buffer for resolving
## ntexture
number of allocated textures
## textureType
type of texture (mjtTexture) (ntexture)
## texture
texture names
## basePlane
all planes from model
## baseMesh
all meshes from model
## baseHField
all hfields from model
## baseBuiltin
all buildin geoms, with quality from model
## baseFontNormal
normal font
## baseFontShadow
shadow font
## baseFontBig
big font
## rangePlane
all planes from model
## rangeMesh
all meshes from model
## rangeHField
all hfields from model
## rangeBuiltin
all builtin geoms, with quality from model
## rangeFont
all characters in font
## nskin
number of skins
## skinvertVBO
skin vertex position VBOs (nskin)
## skinnormalVBO
skin vertex normal VBOs (nskin)
## skintexcoordVBO
skin vertex texture coordinate VBOs (nskin)
## skinfaceVBO
skin face index VBOs (nskin)
## charWidth
character widths: normal and shadow
## charWidthBig
chacarter widths: big
## charHeight
character heights: normal and shadow
## charHeightBig
character heights: big
## glInitialized
is OpenGL initialized
## windowAvailable
is default/window framebuffer available
## windowSamples
number of samples for default/window framebuffer
## windowStereo
is stereo available for default/window framebuffer
## windowDoublebuffer
is default/window framebuffer double buffered
## currentBuffer
currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN
## readPixelFormat
default color pixel format for mjr_readPixels

"""
struct mjrContext_
    lineWidth::Cfloat
    shadowClip::Cfloat
    shadowScale::Cfloat
    fogStart::Cfloat
    fogEnd::Cfloat
    fogRGBA::NTuple{4,Cfloat}
    shadowSize::Cint
    offWidth::Cint
    offHeight::Cint
    offSamples::Cint
    fontScale::Cint
    auxWidth::NTuple{10,Cint}
    auxHeight::NTuple{10,Cint}
    auxSamples::NTuple{10,Cint}
    offFBO::Cuint
    offFBO_r::Cuint
    offColor::Cuint
    offColor_r::Cuint
    offDepthStencil::Cuint
    offDepthStencil_r::Cuint
    shadowFBO::Cuint
    shadowTex::Cuint
    auxFBO::NTuple{10,Cuint}
    auxFBO_r::NTuple{10,Cuint}
    auxColor::NTuple{10,Cuint}
    auxColor_r::NTuple{10,Cuint}
    ntexture::Cint
    textureType::NTuple{100,Cint}
    texture::NTuple{100,Cuint}
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
    charWidth::NTuple{127,Cint}
    charWidthBig::NTuple{127,Cint}
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
const mjtButton = mjtButton_
const mjtEvent = mjtEvent_
const mjtItem = mjtItem_
"""
	mjuiState

# Fields

## nrect
number of rectangles used
## rect
rectangles (index 0: entire window)
## userdata
pointer to user data (for callbacks)
## type
(type mjtEvent)
## left
is left button down
## right
is right button down
## middle
is middle button down
## doubleclick
is last press a double click
## button
which button was pressed (mjtButton)
## buttontime
time of last button press
## x
x position
## y
y position
## dx
x displacement
## dy
y displacement
## sx
x scroll
## sy
y scroll
## control
is control down
## shift
is shift down
## alt
is alt down
## key
which key was pressed
## keytime
time of last key press
## mouserect
which rectangle contains mouse
## dragrect
which rectangle is dragged with mouse
## dragbutton
which button started drag (mjtButton)
## dropcount
number of files dropped
## droppaths
paths to files dropped

"""
struct mjuiState_
    nrect::Cint
    rect::NTuple{25,mjrRect}
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
"""
	mjuiThemeSpacing

# Fields

## total
total width
## scroll
scrollbar width
## label
label width
## section
section gap
## itemside
item side gap
## itemmid
item middle gap
## itemver
item vertical gap
## texthor
text horizontal gap
## textver
text vertical gap
## linescroll
number of pixels to scroll
## samples
number of multisamples

"""
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
"""
	mjuiThemeColor

# Fields

## master
master background
## thumb
scrollbar thumb
## secttitle
section title
## sectfont
section font
## sectsymbol
section symbol
## sectpane
section pane
## shortcut
shortcut background
## fontactive
font active
## fontinactive
font inactive
## decorinactive
decor inactive
## decorinactive2
inactive slider color 2
## button
button
## check
check
## radio
radio
## select
select
## select2
select pane
## slider
slider
## slider2
slider color 2
## edit
edit
## edit2
edit invalid
## cursor
edit cursor

"""
struct mjuiThemeColor_
    master::NTuple{3,Cfloat}
    thumb::NTuple{3,Cfloat}
    secttitle::NTuple{3,Cfloat}
    sectfont::NTuple{3,Cfloat}
    sectsymbol::NTuple{3,Cfloat}
    sectpane::NTuple{3,Cfloat}
    shortcut::NTuple{3,Cfloat}
    fontactive::NTuple{3,Cfloat}
    fontinactive::NTuple{3,Cfloat}
    decorinactive::NTuple{3,Cfloat}
    decorinactive2::NTuple{3,Cfloat}
    button::NTuple{3,Cfloat}
    check::NTuple{3,Cfloat}
    radio::NTuple{3,Cfloat}
    select::NTuple{3,Cfloat}
    select2::NTuple{3,Cfloat}
    slider::NTuple{3,Cfloat}
    slider2::NTuple{3,Cfloat}
    edit::NTuple{3,Cfloat}
    edit2::NTuple{3,Cfloat}
    cursor::NTuple{3,Cfloat}
end
const mjuiThemeColor = mjuiThemeColor_
struct mjuiItemSingle_
    modifier::Cint
    shortcut::Cint
end
struct mjuiItemMulti_
    nelem::Cint
    name::NTuple{35,NTuple{40,Cchar}}
end
struct mjuiItemSlider_
    range::NTuple{2,Cdouble}
    divisions::Cdouble
end
struct mjuiItemEdit_
    nelem::Cint
    range::NTuple{7,NTuple{2,Cdouble}}
end
"""
	mjuiItem

# Fields

## type
type (mjtItem)
## name
name
## state
0: disable, 1: enable, 2+: use predicate
## pdata
data pointer (type-specific)
## sectionid
id of section containing item
## itemid
id of item within section
## rect
rectangle occupied by item

"""
struct mjuiItem_
    data::NTuple{1488,UInt8}
end
const mjuiItem = mjuiItem_
"""
	mjuiSection

# Fields

## name
name
## state
0: closed, 1: open
## modifier
0: none, 1: control, 2: shift; 4: alt
## shortcut
shortcut key; 0: undefined
## nitem
number of items in use
## item
preallocated array of items
## rtitle
rectangle occupied by title
## rcontent
rectangle occupied by content

"""
struct mjuiSection_
    name::NTuple{40,Cchar}
    state::Cint
    modifier::Cint
    shortcut::Cint
    nitem::Cint
    item::NTuple{100,mjuiItem}
    rtitle::mjrRect
    rcontent::mjrRect
end
const mjuiSection = mjuiSection_
"""
	mjUI

# Fields

## spacing
UI theme spacing
## color
UI theme color
## predicate
callback to set item state programmatically
## userdata
pointer to user data (passed to predicate)
## rectid
index of this ui rectangle in mjuiState
## auxid
aux buffer index of this ui
## radiocol
number of radio columns (0 defaults to 2)
## width
width
## height
current heigth
## maxheight
height when all sections open
## scroll
scroll from top of UI
## mousesect
0: none, -1: scroll, otherwise 1+section
## mouseitem
item within section
## mousehelp
help button down: print shortcuts
## editsect
0: none, otherwise 1+section
## edititem
item within section
## editcursor
cursor position
## editscroll
horizontal scroll
## edittext
current text
## editchanged
pointer to changed edit in last mjui_event
## nsect
number of sections in use
## sect
preallocated array of sections

"""
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
    edittext::NTuple{300,Cchar}
    editchanged::Ptr{mjuiItem}
    nsect::Cint
    sect::NTuple{10,mjuiSection}
end
const mjUI = mjUI_
"""
	mjuiDef

# Fields

## type
type (mjtItem); -1: section
## name
name
## state
state
## pdata
pointer to data
## other
string with type-specific properties

"""
struct mjuiDef_
    type::Cint
    name::NTuple{40,Cchar}
    state::Cint
    pdata::Ptr{Cvoid}
    other::NTuple{300,Cchar}
end
const mjuiDef = mjuiDef_
const mjtCatBit = mjtCatBit_
const mjtMouse = mjtMouse_
const mjtPertBit = mjtPertBit_
const mjtCamera = mjtCamera_
const mjtLabel = mjtLabel_
const mjtFrame = mjtFrame_
const mjtVisFlag = mjtVisFlag_
const mjtRndFlag = mjtRndFlag_
const mjtStereo = mjtStereo_
"""
	mjvPerturb

# Fields

## select
selected body id; non-positive: none
## skinselect
selected skin id; negative: none
## active
perturbation bitmask (mjtPertBit)
## active2
secondary perturbation bitmask (mjtPertBit)
## refpos
reference position for selected object
## refquat
reference orientation for selected object
## refselpos
reference position for selection point
## localpos
selection point in object coordinates
## localmass
spatial inertia at selection point
## scale
relative mouse motion-to-space scaling (set by initPerturb)

"""
struct mjvPerturb_
    select::Cint
    skinselect::Cint
    active::Cint
    active2::Cint
    refpos::NTuple{3,mjtNum}
    refquat::NTuple{4,mjtNum}
    refselpos::NTuple{3,mjtNum}
    localpos::NTuple{3,mjtNum}
    localmass::mjtNum
    scale::mjtNum
end
const mjvPerturb = mjvPerturb_
"""
	mjvCamera

# Fields

## type
camera type (mjtCamera)
## fixedcamid
fixed camera id
## trackbodyid
body id to track
## lookat
lookat point
## distance
distance to lookat point or tracked body
## azimuth
camera azimuth (deg)
## elevation
camera elevation (deg)

"""
struct mjvCamera_
    type::Cint
    fixedcamid::Cint
    trackbodyid::Cint
    lookat::NTuple{3,mjtNum}
    distance::mjtNum
    azimuth::mjtNum
    elevation::mjtNum
end
const mjvCamera = mjvCamera_
"""
	mjvGLCamera

# Fields

## pos
position
## forward
forward direction
## up
up direction
## frustum_center
hor. center (left,right set to match aspect)
## frustum_bottom
bottom
## frustum_top
top
## frustum_near
near
## frustum_far
far

"""
struct mjvGLCamera_
    pos::NTuple{3,Cfloat}
    forward::NTuple{3,Cfloat}
    up::NTuple{3,Cfloat}
    frustum_center::Cfloat
    frustum_bottom::Cfloat
    frustum_top::Cfloat
    frustum_near::Cfloat
    frustum_far::Cfloat
end
const mjvGLCamera = mjvGLCamera_
"""
	mjvGeom

# Fields

## type
geom type (mjtGeom)
## dataid
mesh, hfield or plane id; -1: none
## objtype
mujoco object type; mjOBJ_UNKNOWN for decor
## objid
mujoco object id; -1 for decor
## category
visual category
## texid
texture id; -1: no texture
## texuniform
uniform cube mapping
## texcoord
mesh geom has texture coordinates
## segid
segmentation id; -1: not shown
## texrepeat
texture repetition for 2D mapping
## size
size parameters
## pos
Cartesian position
## mat
Cartesian orientation
## rgba
color and transparency
## emission
emission coef
## specular
specular coef
## shininess
shininess coef
## reflectance
reflectance coef
## label
text label
## camdist
distance to camera (used by sorter)
## modelrbound
geom rbound from model, 0 if not model geom
## transparent
treat geom as transparent

"""
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
    texrepeat::NTuple{2,Cfloat}
    size::NTuple{3,Cfloat}
    pos::NTuple{3,Cfloat}
    mat::NTuple{9,Cfloat}
    rgba::NTuple{4,Cfloat}
    emission::Cfloat
    specular::Cfloat
    shininess::Cfloat
    reflectance::Cfloat
    label::NTuple{100,Cchar}
    camdist::Cfloat
    modelrbound::Cfloat
    transparent::mjtByte
end
const mjvGeom = mjvGeom_
"""
	mjvLight

# Fields

## pos
position rel. to body frame
## dir
direction rel. to body frame
## attenuation
OpenGL attenuation (quadratic model)
## cutoff
OpenGL cutoff
## exponent
OpenGL exponent
## ambient
ambient rgb (alpha=1)
## diffuse
diffuse rgb (alpha=1)
## specular
specular rgb (alpha=1)
## headlight
headlight
## directional
directional light
## castshadow
does light cast shadows

"""
struct mjvLight_
    pos::NTuple{3,Cfloat}
    dir::NTuple{3,Cfloat}
    attenuation::NTuple{3,Cfloat}
    cutoff::Cfloat
    exponent::Cfloat
    ambient::NTuple{3,Cfloat}
    diffuse::NTuple{3,Cfloat}
    specular::NTuple{3,Cfloat}
    headlight::mjtByte
    directional::mjtByte
    castshadow::mjtByte
end
const mjvLight = mjvLight_
"""
	mjvOption

# Fields

## label
what objects to label (mjtLabel)
## frame
which frame to show (mjtFrame)
## geomgroup
geom visualization by group
## sitegroup
site visualization by group
## jointgroup
joint visualization by group
## tendongroup
tendon visualization by group
## actuatorgroup
actuator visualization by group
## skingroup
skin visualization by group
## flags
visualization flags (indexed by mjtVisFlag)
## bvh_depth
depth of the bounding volume hierarchy to be visualized

"""
struct mjvOption_
    label::Cint
    frame::Cint
    geomgroup::NTuple{6,mjtByte}
    sitegroup::NTuple{6,mjtByte}
    jointgroup::NTuple{6,mjtByte}
    tendongroup::NTuple{6,mjtByte}
    actuatorgroup::NTuple{6,mjtByte}
    skingroup::NTuple{6,mjtByte}
    flags::NTuple{25,mjtByte}
    bvh_depth::Cint
end
const mjvOption = mjvOption_
"""
	mjvScene

# Fields

## maxgeom
size of allocated geom buffer
## ngeom
number of geoms currently in buffer
## geoms
buffer for geoms (ngeom)
## geomorder
buffer for ordering geoms by distance to camera (ngeom)
## nskin
number of skins
## skinfacenum
number of faces in skin (nskin)
## skinvertadr
address of skin vertices (nskin)
## skinvertnum
number of vertices in skin (nskin)
## skinvert
skin vertex data (nskin)
## skinnormal
skin normal data (nskin)
## nlight
number of lights currently in buffer
## lights
buffer for lights (nlight)
## camera
left and right camera
## enabletransform
enable model transformation
## translate
model translation
## rotate
model quaternion rotation
## scale
model scaling
## stereo
stereoscopic rendering (mjtStereo)
## flags
rendering flags (indexed by mjtRndFlag)
## framewidth
frame pixel width; 0: disable framing
## framergb
frame color

"""
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
    lights::NTuple{100,mjvLight}
    camera::NTuple{2,mjvGLCamera}
    enabletransform::mjtByte
    translate::NTuple{3,Cfloat}
    rotate::NTuple{4,Cfloat}
    scale::Cfloat
    stereo::Cint
    flags::NTuple{10,mjtByte}
    framewidth::Cint
    framergb::NTuple{3,Cfloat}
end
const mjvScene = mjvScene_
"""
	mjvFigure

# Fields

## flg_legend
show legend
## flg_ticklabel
show grid tick labels (x,y)
## flg_extend
automatically extend axis ranges to fit data
## flg_barplot
isolated line segments (i.e. GL_LINES)
## flg_selection
vertical selection line
## flg_symmetric
symmetric y-axis
## linewidth
line width
## gridwidth
grid line width
## gridsize
number of grid points in (x,y)
## gridrgb
grid line rgb
## figurergba
figure color and alpha
## panergba
pane color and alpha
## legendrgba
legend color and alpha
## textrgb
text color
## linergb
line colors
## range
axis ranges; (min>=max) automatic
## xformat
x-tick label format for sprintf
## yformat
y-tick label format for sprintf
## minwidth
string used to determine min y-tick width
## title
figure title; subplots separated with 2+ spaces
## xlabel
x-axis label
## linename
line names for legend
## legendoffset
number of lines to offset legend
## subplot
selected subplot (for title rendering)
## highlight
if point is in legend rect, highlight line
## highlightid
if id>=0 and no point, highlight id
## selection
selection line x-value
## linepnt
number of points in line; (0) disable
## linedata
line data (x,y)
## xaxispixel
range of x-axis in pixels
## yaxispixel
range of y-axis in pixels
## xaxisdata
range of x-axis in data units
## yaxisdata
range of y-axis in data units

"""
struct mjvFigure_
    flg_legend::Cint
    flg_ticklabel::NTuple{2,Cint}
    flg_extend::Cint
    flg_barplot::Cint
    flg_selection::Cint
    flg_symmetric::Cint
    linewidth::Cfloat
    gridwidth::Cfloat
    gridsize::NTuple{2,Cint}
    gridrgb::NTuple{3,Cfloat}
    figurergba::NTuple{4,Cfloat}
    panergba::NTuple{4,Cfloat}
    legendrgba::NTuple{4,Cfloat}
    textrgb::NTuple{3,Cfloat}
    linergb::NTuple{100,NTuple{3,Cfloat}}
    range::NTuple{2,NTuple{2,Cfloat}}
    xformat::NTuple{20,Cchar}
    yformat::NTuple{20,Cchar}
    minwidth::NTuple{20,Cchar}
    title::NTuple{1000,Cchar}
    xlabel::NTuple{100,Cchar}
    linename::NTuple{100,NTuple{100,Cchar}}
    legendoffset::Cint
    subplot::Cint
    highlight::NTuple{2,Cint}
    highlightid::Cint
    selection::Cfloat
    linepnt::NTuple{100,Cint}
    linedata::NTuple{100,NTuple{2000,Cfloat}}
    xaxispixel::NTuple{2,Cint}
    yaxispixel::NTuple{2,Cint}
    xaxisdata::NTuple{2,Cfloat}
    yaxisdata::NTuple{2,Cfloat}
end
const mjvFigure = mjvFigure_
struct var"##Ctag#310"
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
struct var"##Ctag#311"
    warning::NTuple{8,mjWarningStat}
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
"""
	mjvSceneState

# Fields

## nbuffer
size of the buffer in bytes
## buffer
heap-allocated memory for all arrays in this struct
## maxgeom
maximum number of mjvGeom supported by this state object
## plugincache
scratch space for vis geoms inserted by plugins

"""
struct mjvSceneState_
    data::NTuple{10856,UInt8}
end
const mjvSceneState = mjvSceneState_
