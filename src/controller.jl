struct Controller{QP<:AbstractArray, QV<:AbstractArray, SD<:AbstractArray, CA<:AbstractArray}
    model::Model
    data::Data
    qpos::QP
    qvel::QV
    sensors::SD
    control::CA
end


function Controller(model::Model, data::Data)
    qpos = UnsafeArray(data.qpos, (Int(model.nq), 1))
    qvel = UnsafeArray(data.qvel, (Int(model.nv), 1))
    sensors = UnsafeArray(data.sensordata, (Int(model.nsensordata), 1))
    control = UnsafeArray(data.ctrl, (Int(model.nu), 1))

    return Controller(model, data, qpos, qvel, sensors, control)
end