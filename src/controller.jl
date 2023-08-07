struct Controller{QP<:AbstractArray, QV<:AbstractArray, SD<:AbstractArray, CA<:AbstractArray}
    model::Model
    data::Data
    qpos::QP
    qvel::QV
    sensors::SD
    control::CA
end


function Controller(model::Model, data::Data)
    # This is a really dodgy way to access pointers, why are the heap pointers shifted? Is this actually correct or is there an incorrect offset?
    qpos = UnsafeArray(typeof(data.qpos)(UInt64(data.qpos) >> 32), (Int(model.nq),))
    qvel = UnsafeArray(typeof(data.qvel)(UInt64(data.qvel) >> 32), (Int(model.nv),))
    sensors = UnsafeArray(typeof(data.sensordata)(UInt64(data.sensordata) >> 32), (Int(model.nsensordata),))
    control = UnsafeArray(typeof(data.ctrl)(UInt64(data.ctrl) >> 32), (Int(model.nu),))

    return Controller(model, data, qpos, qvel, sensors, control)
end