using CairoMakie
using LinearAlgebra
using MuJoCo
using MuJoCo.LibMuJoCo

init_visualiser()
isplot = false

# Note: follow along with the DeepMind notebook: 
# https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/LQR.ipynb

# TODO: Make this into a Pluto.jl notebook?


# Load humanoid in specific keyframe
model, data = MuJoCo.sample_model_and_data()
reset!(m::Model, d::Data) = LibMuJoCo.mj_resetDataKeyframe(m, d, 1)
reset!(model, data)


################## Get control set-point ##################

# We want to torque required to hold the humanoid in this position
# Just using inverse dynamics won't work yet - there's an unphysical vertical 
# contact force with the ground we want to remove first. Here's a hacky approach.
heights = LinRange(-0.001, 0.001, 2001)
u_vert = zeros(length(heights))
for k in eachindex(heights)

    # Set model in position and assume qacc == 0
    reset!(model, data)
    forward!(model, data)
    data.qacc .= 0

    # Offset the height and check required vertical forces
    data.qpos[3] += heights[k]
    LibMuJoCo.mj_inverse(model, data)
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
reset!(model, data)
forward!(model, data)
data.qacc .= 0
data.qpos[3] += height
qpos0 = vec(copy(data.qpos))
LibMuJoCo.mj_inverse(model, data)
qfrc0 = vec(copy(data.qfrc_inverse))
println("Desired forces qfrc0 acquired")

# Need the corresponding control torque (through the actuators)
M_act = data.actuator_moment # TODO: This should be in column-major not row-major form... bugger
ctrl0 = pinv(M_act)' * qfrc0
println("Control set-point ctrl0 acquired")

# Double-check (note: this works because the humanoid is fully-actuated!)
data.ctrl .= ctrl0
forward!(model, data)
qfrc_test = vec(copy(data.qfrc_actuator))
println("Desired force meets actual? ", qfrc_test .≈ qfrc0)

# # Take a look
# visualise!(model, data)