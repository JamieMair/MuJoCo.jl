using MuJoCo
using LinearAlgebra
using MatrixEquations

init_visualiser()

# Get the model
model = load_model("cartpole.xml")
data = init_data(model)

# Make sure it's in the upright equilibrium
println(data.qpos)
println(data.qvel)

# Number of states and controlled inputs
nx = 2*model.nv
nu = model.nu

# Finite-difference parameters
ϵ = 1e-6
centred = true

# Compute the Jacobians
A = mj_zeros(nx, nx)
B = mj_zeros(nx, nu)
mjd_transitionFD(model, data, ϵ, centred, A, B, nothing, nothing)

# Design LQR controller
Q = diagm([1, 10, 1, 5])
R = diagm([1])
S = zeros(nx, nu)
_, _, K, _ = ared(A,B,R,Q,S)

# The controller function
function lqr_balance!(m::Model, d::Data)
    state = vcat(d.qpos, d.qvel)
    d.ctrl .= -K * state
end

# Simulate it
visualise!(model, data; controller=lqr_balance!)