using MuJoCo
init_visualiser()
# MuJoCo.Visualiser.test_visualiser()

# Load model in specific state
model = load_model("cartpole.xml")
data = init_data(model)
data.qpos[2] = 0.01

# Simulate for some time and log states
tmax = 500
nx = model.nq + model.nv + model.na
states = zeros(nx, tmax)
for t in 1:tmax
    states[:,t] = get_physics_state(model, data)
    step!(model, data)
end

# Simulate with a controller
reset!(model, data)
ctrl_states = zeros(nx, tmax)
for t in 1:tmax
    ctrl_states[:,t] = get_physics_state(model, data)
    data.ctrl .= 2*randn()
    step!(model, data)
end

# Try trajectory mode
reset!(model, data)
visualise!(model, data, trajectories = [states, ctrl_states])