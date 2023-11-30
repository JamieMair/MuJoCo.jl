using CairoMakie
using LinearAlgebra
using MatrixEquations: ared
using MuJoCo
import MuJoCo as MJ

init_visualiser()
isplot = false

# Note: follow along with the DeepMind notebook: 
# https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/LQR.ipynb

# Load humanoid in specific keyframe
model, data = MuJoCo.sample_model_and_data()
const keyframe = 2 # Stand on one leg is the second keyframe.
resetkey!(model, data, keyframe)


################## Get control set-point ##################

# We want to torque required to hold the humanoid in this position
# Just using inverse dynamics won't work yet - there's an unphysical vertical 
# contact force with the ground we want to remove first. Here's a hacky approach.
heights = LinRange(-0.001, 0.001, 2001)
u_vert = zeros(length(heights))
for k in eachindex(heights)

    # Set model in position and assume qacc == 0
    resetkey!(model, data, keyframe)
    forward!(model, data)
    data.qacc .= 0

    # Offset the height and check required vertical forces
    data.qpos[3] += heights[k]
    mj_inverse(model, data)
    u_vert[k] = data.qfrc_inverse[3] # 3 -> z-force
end

# Find height corresponding to minimum ficticous force (best offset)
height = heights[argmin(abs.(u_vert))]

# Plot the relationship just to see - compare force to weight of humanoid
weight = sum(model.body_mass) * norm(model.opt.gravity)

fig = Figure(resolution=(600,400))
ax = Axis(fig[1,1], xlabel="Vertical offsets (mm)", ylabel="Vertical force (N)")
lines!(ax, heights*1000, u_vert)
lines!(ax, heights*1000, weight*ones(length(heights)), linestyle=:dash)
lines!(ax, [height, height]*1000, [minimum(u_vert), maximum(u_vert)], linestyle=:dash)
isplot && display(fig)

# We'll use the best-choice offset to get our required ID forces and save q0
resetkey!(model, data, keyframe)
forward!(model, data)
data.qacc .= 0
data.qpos[3] += height
qpos0 = vec(copy(data.qpos))
mj_inverse(model, data)
qfrc0 = vec(copy(data.qfrc_inverse))
println("Desired forces qfrc0 acquired")

# Need the corresponding control torque (through the actuators)
M_act = data.actuator_moment
ctrl0 = pinv(M_act)' * qfrc0
println("Control set-point ctrl0 acquired")

# Double-check (note: this works because the humanoid is fully-actuated!)
data.ctrl .= ctrl0
forward!(model, data)
qfrc_test = vec(copy(data.qfrc_actuator))
println("Desired force meets actual? ", all((qfrc_test .≈ qfrc0)[7:end]))

# Run the simulation
reset!(model, data)
data.qpos .= qpos0
data.ctrl .= ctrl0

isplot && visualise!(model, data)


################## LQR Design ##################

# Useful dimensions
nu = model.nu
nv = model.nv

# R-matrix just identity
R = Matrix{Float64}(I, nu, nu)

# Body IDs
torso = MJ.body(model, "torso")
left_foot = MJ.body(model, "foot_left")

# Get Jacobian for torso CoM
reset!(model, data)
data.qpos .= qpos0
forward!(model, data)
jac_com = mj_zeros(3, nv)
mj_jacSubtreeCom(model, data, jac_com, torso.id)

# Get (left) foot Jacobian for balancing
jac_foot = mj_zeros(3, nv)
mj_jacBodyCom(model, data, jac_foot, nothing, left_foot.id)

# Design Q-matrix to balance CoM over foot
jac_diff = jac_com .- jac_foot
Qbalance = jac_diff' * jac_diff

# Now we include a cost on joints deviating from the steady-state.
# Torso already sorted. Left leg should remain rigid. Other joints can move for balance.

# Get indices into relevant sets of joints.
free_dofs = 1:6
body_dofs = 7:nv

# Get all the joints using a list comprehension. We add one to the raw ID to get the Julia 1-based index of the joint.
abdomen_dofs = [jnt.id+1 for jnt in MJ.joints(model) if occursin("abdomen", jnt.name)]
left_leg_dofs = [jnt.id+1 for jnt in MJ.joints(model) if occursin("left", jnt.name) && any(occursin(part, jnt.name) for part in ("hip", "knee", "ankle"))]

balance_dofs = vcat(abdomen_dofs, left_leg_dofs)
other_dofs = setdiff(body_dofs, balance_dofs)

# Cost coefficients
balance_cost       = 1000       # CoM units large, keep it still
balance_joint_cost = 3          # Joints can move a bit and still balance
other_joint_cost   = 0.3        # Other joints can do whatever

# Construct joint Q matrix
Qjoint = Matrix{Float64}(I, nv, nv)
Qjoint[free_dofs, free_dofs] *= 0
Qjoint[balance_dofs, balance_dofs] *= balance_joint_cost
Qjoint[other_dofs, other_dofs] *= other_joint_cost

# Total Q-matrix
Qpos = balance_cost*Qbalance + Qjoint
Q = [Qpos zeros(nv,nv); zeros(nv, 2nv)]  + (1e-10)I # Add ϵI for positive definite Q

# Get A and B matrices from Jacobian
reset!(model, data)
data.ctrl .= ctrl0
data.qpos .= qpos0

A = mj_zeros(2nv, 2nv)
B = mj_zeros(2nv, nu)
ϵ = 1e-6
centred = true
mjd_transitionFD(model, data, ϵ, centred, A, B, nothing, nothing)

# Solve LQR with MatrixEquations.jl (faster than loading ControlSystems.jl)
S = zeros(size(Q,1), size(R,1))
_, _, K, _ = ared(A,B,R,Q,S)

# Write the LQR function
function humanoid_ctrl!(m::Model, d::Data)

    # Get difference in states qpos - qpos0 (this function does quaternion diff)
    Δq = zeros(nv)
    mj_differentiatePos(m, Δq, 1, qpos0, d.qpos)
    Δx = vcat(Δq, data.qvel)

    # Compute controls with LQR
    data.ctrl .= (ctrl0 .- K*Δx)
    return nothing
end

# Initialise the humanoid and visualise
reset!(model, data)
data.qpos .= qpos0
visualise!(model, data, controller=humanoid_ctrl!)
