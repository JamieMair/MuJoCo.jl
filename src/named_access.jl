module NamedAccess
import ..LibMuJoCo
using UnsafeArrays
import ..Data
import ..Model
include("named_model.jl")
export light, cam, actuator, body, geom, jnt, sensor, site, tendon, eq, key, numeric, mat, tex, pair, hfield, tuple, skin, exclude, mesh, camera, ten, joint, equality, texture, keyframe, material
struct DataActuator
    data::Data
    index::Int
end
struct DataTendon
    data::Data
    index::Int
end
struct DataSite
    data::Data
    index::Int
end
struct DataSensor
    data::Data
    index::Int
end
struct DataBody
    data::Data
    index::Int
end
struct DataCamera
    data::Data
    index::Int
end
struct DataJoint
    data::Data
    index::Int
end
struct DataLight
    data::Data
    index::Int
end
struct DataGeom
    data::Data
    index::Int
end
function light(data::Data, index::Integer)
    return DataLight(data, index)
end
function light(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_LIGHT, name)
    return DataLight(data, index)
end
function light(data::NamedData, index::Integer)
    return DataLight(getfield(data, :data), index)
end
function light(data::NamedData, name::Symbol)
    index = index_by_name(data, :light, name)
    return DataLight(getfield(data, :data), index)
end
function Base.propertynames(::DataLight)
    (:id, :name, :xpos, :xdir)
end
function Base.getproperty(x::DataLight, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :xpos && return view(data.light_xpos, index + 1, Base.OneTo(3))
    f === :xdir && return view(data.light_xdir, index + 1, Base.OneTo(3))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_LIGHT, index)
    error("Could not find the property: " * string(f))
end
function cam(data::Data, index::Integer)
    return DataCamera(data, index)
end
function cam(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_CAMERA, name)
    return DataCamera(data, index)
end
function cam(data::NamedData, index::Integer)
    return DataCamera(getfield(data, :data), index)
end
function cam(data::NamedData, name::Symbol)
    index = index_by_name(data, :cam, name)
    return DataCamera(getfield(data, :data), index)
end
function Base.propertynames(::DataCamera)
    (:id, :name, :xpos, :xmat)
end
function Base.getproperty(x::DataCamera, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :xpos && return view(data.cam_xpos, index + 1, Base.OneTo(3))
    f === :xmat && return view(data.cam_xmat, index + 1, Base.OneTo(9))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_CAMERA, index)
    error("Could not find the property: " * string(f))
end
function actuator(data::Data, index::Integer)
    return DataActuator(data, index)
end
function actuator(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_ACTUATOR, name)
    return DataActuator(data, index)
end
function actuator(data::NamedData, index::Integer)
    return DataActuator(getfield(data, :data), index)
end
function actuator(data::NamedData, name::Symbol)
    index = index_by_name(data, :actuator, name)
    return DataActuator(getfield(data, :data), index)
end
function Base.propertynames(::DataActuator)
    (:id, :name, :ctrl, :length, :moment, :velocity, :force)
end
function Base.getproperty(x::DataActuator, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :ctrl && return view(data.ctrl, index + 1, Base.OneTo(1))
    f === :length && return view(data.actuator_length, index + 1, Base.OneTo(1))
    f === :moment && return view(data.actuator_moment, index + 1, Base.OneTo(model.nv))
    f === :velocity && return view(data.actuator_velocity, index + 1, Base.OneTo(1))
    f === :force && return view(data.actuator_force, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_ACTUATOR, index)
    error("Could not find the property: " * string(f))
end
function body(data::Data, index::Integer)
    return DataBody(data, index)
end
function body(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_BODY, name)
    return DataBody(data, index)
end
function body(data::NamedData, index::Integer)
    return DataBody(getfield(data, :data), index)
end
function body(data::NamedData, name::Symbol)
    index = index_by_name(data, :body, name)
    return DataBody(getfield(data, :data), index)
end
function Base.propertynames(::DataBody)
    (:id, :name, :applied, :xpos, :xquat, :xmat, :xipos, :ximat, :com, :cinert, :crb, :cvel, :linvel, :angmom, :cacc, :int, :ext)
end
function Base.getproperty(x::DataBody, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :applied && return view(data.xfrc_applied, index + 1, Base.OneTo(6))
    f === :xpos && return view(data.xpos, index + 1, Base.OneTo(3))
    f === :xquat && return view(data.xquat, index + 1, Base.OneTo(4))
    f === :xmat && return view(data.xmat, index + 1, Base.OneTo(9))
    f === :xipos && return view(data.xipos, index + 1, Base.OneTo(3))
    f === :ximat && return view(data.ximat, index + 1, Base.OneTo(9))
    f === :com && return view(data.subtree_com, index + 1, Base.OneTo(3))
    f === :cinert && return view(data.cinert, index + 1, Base.OneTo(10))
    f === :crb && return view(data.crb, index + 1, Base.OneTo(10))
    f === :cvel && return view(data.cvel, index + 1, Base.OneTo(6))
    f === :linvel && return view(data.subtree_linvel, index + 1, Base.OneTo(3))
    f === :angmom && return view(data.subtree_angmom, index + 1, Base.OneTo(3))
    f === :cacc && return view(data.cacc, index + 1, Base.OneTo(6))
    f === :int && return view(data.cfrc_int, index + 1, Base.OneTo(6))
    f === :ext && return view(data.cfrc_ext, index + 1, Base.OneTo(6))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_BODY, index)
    error("Could not find the property: " * string(f))
end
function geom(data::Data, index::Integer)
    return DataGeom(data, index)
end
function geom(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_GEOM, name)
    return DataGeom(data, index)
end
function geom(data::NamedData, index::Integer)
    return DataGeom(getfield(data, :data), index)
end
function geom(data::NamedData, name::Symbol)
    index = index_by_name(data, :geom, name)
    return DataGeom(getfield(data, :data), index)
end
function Base.propertynames(::DataGeom)
    (:id, :name, :xpos, :xmat)
end
function Base.getproperty(x::DataGeom, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :xpos && return view(data.geom_xpos, index + 1, Base.OneTo(3))
    f === :xmat && return view(data.geom_xmat, index + 1, Base.OneTo(9))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_GEOM, index)
    error("Could not find the property: " * string(f))
end
function jnt(data::Data, index::Integer)
    return DataJoint(data, index)
end
function jnt(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_JOINT, name)
    return DataJoint(data, index)
end
function jnt(data::NamedData, index::Integer)
    return DataJoint(getfield(data, :data), index)
end
function jnt(data::NamedData, name::Symbol)
    index = index_by_name(data, :jnt, name)
    return DataJoint(getfield(data, :data), index)
end
function Base.propertynames(::DataJoint)
    (:id, :name, :xanchor, :xaxis)
end
function Base.getproperty(x::DataJoint, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :xanchor && return view(data.xanchor, index + 1, Base.OneTo(3))
    f === :xaxis && return view(data.xaxis, index + 1, Base.OneTo(3))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_JOINT, index)
    error("Could not find the property: " * string(f))
end
function sensor(data::Data, index::Integer)
    return DataSensor(data, index)
end
function sensor(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_SENSOR, name)
    return DataSensor(data, index)
end
function sensor(data::NamedData, index::Integer)
    return DataSensor(getfield(data, :data), index)
end
function sensor(data::NamedData, name::Symbol)
    index = index_by_name(data, :sensor, name)
    return DataSensor(getfield(data, :data), index)
end
function Base.propertynames(::DataSensor)
    (:id, :name)
end
function Base.getproperty(x::DataSensor, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_SENSOR, index)
    error("Could not find the property: " * string(f))
end
function site(data::Data, index::Integer)
    return DataSite(data, index)
end
function site(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_SITE, name)
    return DataSite(data, index)
end
function site(data::NamedData, index::Integer)
    return DataSite(getfield(data, :data), index)
end
function site(data::NamedData, name::Symbol)
    index = index_by_name(data, :site, name)
    return DataSite(getfield(data, :data), index)
end
function Base.propertynames(::DataSite)
    (:id, :name, :xpos, :xmat)
end
function Base.getproperty(x::DataSite, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :xpos && return view(data.site_xpos, index + 1, Base.OneTo(3))
    f === :xmat && return view(data.site_xmat, index + 1, Base.OneTo(9))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_SITE, index)
    error("Could not find the property: " * string(f))
end
function tendon(data::Data, index::Integer)
    return DataTendon(data, index)
end
function tendon(data::Data, name::String)
    index = index_by_name(data, LibMuJoCo.mjOBJ_TENDON, name)
    return DataTendon(data, index)
end
function tendon(data::NamedData, index::Integer)
    return DataTendon(getfield(data, :data), index)
end
function tendon(data::NamedData, name::Symbol)
    index = index_by_name(data, :tendon, name)
    return DataTendon(getfield(data, :data), index)
end
function Base.propertynames(::DataTendon)
    (:id, :name, :wrapadr, :wrapnum, :J_rownnz, :J_rowadr, :J_colind, :length, :J, :velocity)
end
function Base.getproperty(x::DataTendon, f::Symbol)
    data = getfield(x, :data)
    model = getfield(data, :model)
    index = getfield(x, :index)
    f === :wrapadr && return view(data.ten_wrapadr, index + 1, Base.OneTo(1))
    f === :wrapnum && return view(data.ten_wrapnum, index + 1, Base.OneTo(1))
    f === :J_rownnz && return view(data.ten_J_rownnz, index + 1, Base.OneTo(1))
    f === :J_rowadr && return view(data.ten_J_rowadr, index + 1, Base.OneTo(1))
    f === :J_colind && return view(data.ten_J_colind, index + 1, Base.OneTo(model.nv))
    f === :length && return view(data.ten_length, index + 1, Base.OneTo(1))
    f === :J && return view(data.ten_J, index + 1, Base.OneTo(model.nv))
    f === :velocity && return view(data.ten_velocity, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_TENDON, index)
    error("Could not find the property: " * string(f))
end
struct ModelTexture
    model::Model
    index::Int
end
struct ModelSensor
    model::Model
    index::Int
end
struct ModelLight
    model::Model
    index::Int
end
struct ModelJoint
    model::Model
    index::Int
end
struct ModelEquality
    model::Model
    index::Int
end
struct ModelPair
    model::Model
    index::Int
end
struct ModelTuple
    model::Model
    index::Int
end
struct ModelBody
    model::Model
    index::Int
end
struct ModelCamera
    model::Model
    index::Int
end
struct ModelExclude
    model::Model
    index::Int
end
struct ModelKeyframe
    model::Model
    index::Int
end
struct ModelHfield
    model::Model
    index::Int
end
struct ModelActuator
    model::Model
    index::Int
end
struct ModelTendon
    model::Model
    index::Int
end
struct ModelSite
    model::Model
    index::Int
end
struct ModelMaterial
    model::Model
    index::Int
end
struct ModelSkin
    model::Model
    index::Int
end
struct ModelMesh
    model::Model
    index::Int
end
struct ModelNumeric
    model::Model
    index::Int
end
struct ModelGeom
    model::Model
    index::Int
end
function eq(model::Model, index::Integer)
    return ModelEquality(model, index)
end
function eq(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_EQUALITY, name)
    return ModelEquality(model, index)
end
function eq(model::NamedModel, index::Integer)
    return ModelEquality(getfield(model, :model), index)
end
function eq(model::NamedModel, name::Symbol)
    index = index_by_name(model, :eq, name)
    return ModelEquality(getfield(model, :model), index)
end
function Base.propertynames(::ModelEquality)
    (:id, :name, :type, :obj1id, :obj2id, :active, :solref, :solimp, :data)
end
function Base.getproperty(x::ModelEquality, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :type && return view(model.eq_type, index + 1, Base.OneTo(1))
    f === :obj1id && return view(model.eq_obj1id, index + 1, Base.OneTo(1))
    f === :obj2id && return view(model.eq_obj2id, index + 1, Base.OneTo(1))
    f === :active && return view(model.eq_active, index + 1, Base.OneTo(1))
    f === :solref && return begin
                size_arr = Int(LibMuJoCo.mjNREF)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.eq_solref + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :solimp && return begin
                size_arr = Int(LibMuJoCo.mjNIMP)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.eq_solimp + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :data && return begin
                size_arr = Int(LibMuJoCo.mjNEQDATA)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.eq_data + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_EQUALITY, index)
    error("Could not find the property: " * string(f))
end
function key(model::Model, index::Integer)
    return ModelKeyframe(model, index)
end
function key(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_KEY, name)
    return ModelKeyframe(model, index)
end
function key(model::NamedModel, index::Integer)
    return ModelKeyframe(getfield(model, :model), index)
end
function key(model::NamedModel, name::Symbol)
    index = index_by_name(model, :key, name)
    return ModelKeyframe(getfield(model, :model), index)
end
function Base.propertynames(::ModelKeyframe)
    (:id, :name, :time, :qpos, :qvel, :act, :mpos, :mquat)
end
function Base.getproperty(x::ModelKeyframe, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :time && return view(model.key_time, index + 1, Base.OneTo(1))
    f === :qpos && return view(model.key_qpos, index + 1, Base.OneTo(model.nq))
    f === :qvel && return view(model.key_qvel, index + 1, Base.OneTo(model.nv))
    f === :act && return view(model.key_act, index + 1, Base.OneTo(model.na))
    f === :mpos && return view(model.key_mpos, index + 1, Base.OneTo(model.nmocap * 3))
    f === :mquat && return view(model.key_mquat, index + 1, Base.OneTo(model.nmocap * 4))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_KEY, index)
    error("Could not find the property: " * string(f))
end
function geom(model::Model, index::Integer)
    return ModelGeom(model, index)
end
function geom(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_GEOM, name)
    return ModelGeom(model, index)
end
function geom(model::NamedModel, index::Integer)
    return ModelGeom(getfield(model, :model), index)
end
function geom(model::NamedModel, name::Symbol)
    index = index_by_name(model, :geom, name)
    return ModelGeom(getfield(model, :model), index)
end
function Base.propertynames(::ModelGeom)
    (:id, :name, :type, :contype, :conaffinity, :condim, :bodyid, :dataid, :matid, :group, :priority, :sameframe, :solmix, :solref, :solimp, :size, :rbound, :pos, :quat, :friction, :margin, :gap, :user, :rgba)
end
function Base.getproperty(x::ModelGeom, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :type && return view(model.geom_type, index + 1, Base.OneTo(1))
    f === :contype && return view(model.geom_contype, index + 1, Base.OneTo(1))
    f === :conaffinity && return view(model.geom_conaffinity, index + 1, Base.OneTo(1))
    f === :condim && return view(model.geom_condim, index + 1, Base.OneTo(1))
    f === :bodyid && return view(model.geom_bodyid, index + 1, Base.OneTo(1))
    f === :dataid && return view(model.geom_dataid, index + 1, Base.OneTo(1))
    f === :matid && return view(model.geom_matid, index + 1, Base.OneTo(1))
    f === :group && return view(model.geom_group, index + 1, Base.OneTo(1))
    f === :priority && return view(model.geom_priority, index + 1, Base.OneTo(1))
    f === :sameframe && return view(model.geom_sameframe, index + 1, Base.OneTo(1))
    f === :solmix && return view(model.geom_solmix, index + 1, Base.OneTo(1))
    f === :solref && return begin
                size_arr = Int(LibMuJoCo.mjNREF)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.geom_solref + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :solimp && return begin
                size_arr = Int(LibMuJoCo.mjNIMP)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.geom_solimp + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :size && return view(model.geom_size, index + 1, Base.OneTo(3))
    f === :rbound && return view(model.geom_rbound, index + 1, Base.OneTo(1))
    f === :pos && return view(model.geom_pos, index + 1, Base.OneTo(3))
    f === :quat && return view(model.geom_quat, index + 1, Base.OneTo(4))
    f === :friction && return view(model.geom_friction, index + 1, Base.OneTo(3))
    f === :margin && return view(model.geom_margin, index + 1, Base.OneTo(1))
    f === :gap && return view(model.geom_gap, index + 1, Base.OneTo(1))
    f === :user && return view(model.geom_user, index + 1, Base.OneTo(model.nuser_geom))
    f === :rgba && return view(model.geom_rgba, index + 1, Base.OneTo(4))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_GEOM, index)
    error("Could not find the property: " * string(f))
end
function numeric(model::Model, index::Integer)
    return ModelNumeric(model, index)
end
function numeric(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_NUMERIC, name)
    return ModelNumeric(model, index)
end
function numeric(model::NamedModel, index::Integer)
    return ModelNumeric(getfield(model, :model), index)
end
function numeric(model::NamedModel, name::Symbol)
    index = index_by_name(model, :numeric, name)
    return ModelNumeric(getfield(model, :model), index)
end
function Base.propertynames(::ModelNumeric)
    (:id, :name, :adr, :size)
end
function Base.getproperty(x::ModelNumeric, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :adr && return view(model.numeric_adr, index + 1, Base.OneTo(1))
    f === :size && return view(model.numeric_size, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_NUMERIC, index)
    error("Could not find the property: " * string(f))
end
function sensor(model::Model, index::Integer)
    return ModelSensor(model, index)
end
function sensor(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_SENSOR, name)
    return ModelSensor(model, index)
end
function sensor(model::NamedModel, index::Integer)
    return ModelSensor(getfield(model, :model), index)
end
function sensor(model::NamedModel, name::Symbol)
    index = index_by_name(model, :sensor, name)
    return ModelSensor(getfield(model, :model), index)
end
function Base.propertynames(::ModelSensor)
    (:id, :name, :type, :datatype, :needstage, :objtype, :objid, :reftype, :refid, :dim, :adr, :cutoff, :noise, :user)
end
function Base.getproperty(x::ModelSensor, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :type && return view(model.sensor_type, index + 1, Base.OneTo(1))
    f === :datatype && return view(model.sensor_datatype, index + 1, Base.OneTo(1))
    f === :needstage && return view(model.sensor_needstage, index + 1, Base.OneTo(1))
    f === :objtype && return view(model.sensor_objtype, index + 1, Base.OneTo(1))
    f === :objid && return view(model.sensor_objid, index + 1, Base.OneTo(1))
    f === :reftype && return view(model.sensor_reftype, index + 1, Base.OneTo(1))
    f === :refid && return view(model.sensor_refid, index + 1, Base.OneTo(1))
    f === :dim && return view(model.sensor_dim, index + 1, Base.OneTo(1))
    f === :adr && return view(model.sensor_adr, index + 1, Base.OneTo(1))
    f === :cutoff && return view(model.sensor_cutoff, index + 1, Base.OneTo(1))
    f === :noise && return view(model.sensor_noise, index + 1, Base.OneTo(1))
    f === :user && return view(model.sensor_user, index + 1, Base.OneTo(model.nuser_sensor))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_SENSOR, index)
    error("Could not find the property: " * string(f))
end
function mat(model::Model, index::Integer)
    return ModelMaterial(model, index)
end
function mat(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_MATERIAL, name)
    return ModelMaterial(model, index)
end
function mat(model::NamedModel, index::Integer)
    return ModelMaterial(getfield(model, :model), index)
end
function mat(model::NamedModel, name::Symbol)
    index = index_by_name(model, :mat, name)
    return ModelMaterial(getfield(model, :model), index)
end
function Base.propertynames(::ModelMaterial)
    (:id, :name, :texid, :texuniform, :texrepeat, :emission, :specular, :shininess, :reflectance, :rgba)
end
function Base.getproperty(x::ModelMaterial, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :texid && return view(model.mat_texid, index + 1, Base.OneTo(1))
    f === :texuniform && return view(model.mat_texuniform, index + 1, Base.OneTo(1))
    f === :texrepeat && return view(model.mat_texrepeat, index + 1, Base.OneTo(2))
    f === :emission && return view(model.mat_emission, index + 1, Base.OneTo(1))
    f === :specular && return view(model.mat_specular, index + 1, Base.OneTo(1))
    f === :shininess && return view(model.mat_shininess, index + 1, Base.OneTo(1))
    f === :reflectance && return view(model.mat_reflectance, index + 1, Base.OneTo(1))
    f === :rgba && return view(model.mat_rgba, index + 1, Base.OneTo(4))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_MATERIAL, index)
    error("Could not find the property: " * string(f))
end
function tex(model::Model, index::Integer)
    return ModelTexture(model, index)
end
function tex(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_TEXTURE, name)
    return ModelTexture(model, index)
end
function tex(model::NamedModel, index::Integer)
    return ModelTexture(getfield(model, :model), index)
end
function tex(model::NamedModel, name::Symbol)
    index = index_by_name(model, :tex, name)
    return ModelTexture(getfield(model, :model), index)
end
function Base.propertynames(::ModelTexture)
    (:id, :name, :type, :height, :width, :adr)
end
function Base.getproperty(x::ModelTexture, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :type && return view(model.tex_type, index + 1, Base.OneTo(1))
    f === :height && return view(model.tex_height, index + 1, Base.OneTo(1))
    f === :width && return view(model.tex_width, index + 1, Base.OneTo(1))
    f === :adr && return view(model.tex_adr, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_TEXTURE, index)
    error("Could not find the property: " * string(f))
end
function actuator(model::Model, index::Integer)
    return ModelActuator(model, index)
end
function actuator(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_ACTUATOR, name)
    return ModelActuator(model, index)
end
function actuator(model::NamedModel, index::Integer)
    return ModelActuator(getfield(model, :model), index)
end
function actuator(model::NamedModel, name::Symbol)
    index = index_by_name(model, :actuator, name)
    return ModelActuator(getfield(model, :model), index)
end
function Base.propertynames(::ModelActuator)
    (:id, :name, :trntype, :dyntype, :gaintype, :biastype, :trnid, :actadr, :actnum, :group, :ctrllimited, :forcelimited, :actlimited, :dynprm, :gainprm, :biasprm, :ctrlrange, :forcerange, :actrange, :gear, :cranklength, :acc0, :length0, :lengthrange, :user)
end
function Base.getproperty(x::ModelActuator, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :trntype && return view(model.actuator_trntype, index + 1, Base.OneTo(1))
    f === :dyntype && return view(model.actuator_dyntype, index + 1, Base.OneTo(1))
    f === :gaintype && return view(model.actuator_gaintype, index + 1, Base.OneTo(1))
    f === :biastype && return view(model.actuator_biastype, index + 1, Base.OneTo(1))
    f === :trnid && return view(model.actuator_trnid, index + 1, Base.OneTo(2))
    f === :actadr && return view(model.actuator_actadr, index + 1, Base.OneTo(1))
    f === :actnum && return view(model.actuator_actnum, index + 1, Base.OneTo(1))
    f === :group && return view(model.actuator_group, index + 1, Base.OneTo(1))
    f === :ctrllimited && return view(model.actuator_ctrllimited, index + 1, Base.OneTo(1))
    f === :forcelimited && return view(model.actuator_forcelimited, index + 1, Base.OneTo(1))
    f === :actlimited && return view(model.actuator_actlimited, index + 1, Base.OneTo(1))
    f === :dynprm && return begin
                size_arr = Int(LibMuJoCo.mjNDYN)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.actuator_dynprm + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :gainprm && return begin
                size_arr = Int(LibMuJoCo.mjNGAIN)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.actuator_gainprm + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :biasprm && return begin
                size_arr = Int(LibMuJoCo.mjNBIAS)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.actuator_biasprm + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :ctrlrange && return view(model.actuator_ctrlrange, index + 1, Base.OneTo(2))
    f === :forcerange && return view(model.actuator_forcerange, index + 1, Base.OneTo(2))
    f === :actrange && return view(model.actuator_actrange, index + 1, Base.OneTo(2))
    f === :gear && return view(model.actuator_gear, index + 1, Base.OneTo(6))
    f === :cranklength && return view(model.actuator_cranklength, index + 1, Base.OneTo(1))
    f === :acc0 && return view(model.actuator_acc0, index + 1, Base.OneTo(1))
    f === :length0 && return view(model.actuator_length0, index + 1, Base.OneTo(1))
    f === :lengthrange && return view(model.actuator_lengthrange, index + 1, Base.OneTo(2))
    f === :user && return view(model.actuator_user, index + 1, Base.OneTo(model.nuser_actuator))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_ACTUATOR, index)
    error("Could not find the property: " * string(f))
end
function site(model::Model, index::Integer)
    return ModelSite(model, index)
end
function site(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_SITE, name)
    return ModelSite(model, index)
end
function site(model::NamedModel, index::Integer)
    return ModelSite(getfield(model, :model), index)
end
function site(model::NamedModel, name::Symbol)
    index = index_by_name(model, :site, name)
    return ModelSite(getfield(model, :model), index)
end
function Base.propertynames(::ModelSite)
    (:id, :name, :type, :bodyid, :matid, :group, :sameframe, :size, :pos, :quat, :user, :rgba)
end
function Base.getproperty(x::ModelSite, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :type && return view(model.site_type, index + 1, Base.OneTo(1))
    f === :bodyid && return view(model.site_bodyid, index + 1, Base.OneTo(1))
    f === :matid && return view(model.site_matid, index + 1, Base.OneTo(1))
    f === :group && return view(model.site_group, index + 1, Base.OneTo(1))
    f === :sameframe && return view(model.site_sameframe, index + 1, Base.OneTo(1))
    f === :size && return view(model.site_size, index + 1, Base.OneTo(3))
    f === :pos && return view(model.site_pos, index + 1, Base.OneTo(3))
    f === :quat && return view(model.site_quat, index + 1, Base.OneTo(4))
    f === :user && return view(model.site_user, index + 1, Base.OneTo(model.nuser_site))
    f === :rgba && return view(model.site_rgba, index + 1, Base.OneTo(4))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_SITE, index)
    error("Could not find the property: " * string(f))
end
function pair(model::Model, index::Integer)
    return ModelPair(model, index)
end
function pair(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_PAIR, name)
    return ModelPair(model, index)
end
function pair(model::NamedModel, index::Integer)
    return ModelPair(getfield(model, :model), index)
end
function pair(model::NamedModel, name::Symbol)
    index = index_by_name(model, :pair, name)
    return ModelPair(getfield(model, :model), index)
end
function Base.propertynames(::ModelPair)
    (:id, :name, :dim, :geom1, :geom2, :signature, :solref, :solimp, :margin, :gap, :friction)
end
function Base.getproperty(x::ModelPair, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :dim && return view(model.pair_dim, index + 1, Base.OneTo(1))
    f === :geom1 && return view(model.pair_geom1, index + 1, Base.OneTo(1))
    f === :geom2 && return view(model.pair_geom2, index + 1, Base.OneTo(1))
    f === :signature && return view(model.pair_signature, index + 1, Base.OneTo(1))
    f === :solref && return begin
                size_arr = Int(LibMuJoCo.mjNREF)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.pair_solref + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :solimp && return begin
                size_arr = Int(LibMuJoCo.mjNIMP)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.pair_solimp + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :margin && return view(model.pair_margin, index + 1, Base.OneTo(1))
    f === :gap && return view(model.pair_gap, index + 1, Base.OneTo(1))
    f === :friction && return view(model.pair_friction, index + 1, Base.OneTo(5))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_PAIR, index)
    error("Could not find the property: " * string(f))
end
function hfield(model::Model, index::Integer)
    return ModelHfield(model, index)
end
function hfield(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_HFIELD, name)
    return ModelHfield(model, index)
end
function hfield(model::NamedModel, index::Integer)
    return ModelHfield(getfield(model, :model), index)
end
function hfield(model::NamedModel, name::Symbol)
    index = index_by_name(model, :hfield, name)
    return ModelHfield(getfield(model, :model), index)
end
function Base.propertynames(::ModelHfield)
    (:id, :name, :size, :nrow, :ncol, :adr)
end
function Base.getproperty(x::ModelHfield, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :size && return view(model.hfield_size, index + 1, Base.OneTo(4))
    f === :nrow && return view(model.hfield_nrow, index + 1, Base.OneTo(1))
    f === :ncol && return view(model.hfield_ncol, index + 1, Base.OneTo(1))
    f === :adr && return view(model.hfield_adr, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_HFIELD, index)
    error("Could not find the property: " * string(f))
end
function light(model::Model, index::Integer)
    return ModelLight(model, index)
end
function light(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_LIGHT, name)
    return ModelLight(model, index)
end
function light(model::NamedModel, index::Integer)
    return ModelLight(getfield(model, :model), index)
end
function light(model::NamedModel, name::Symbol)
    index = index_by_name(model, :light, name)
    return ModelLight(getfield(model, :model), index)
end
function Base.propertynames(::ModelLight)
    (:id, :name, :mode, :bodyid, :targetbodyid, :directional, :castshadow, :active, :pos, :dir, :poscom0, :pos0, :dir0, :attenuation, :cutoff, :exponent, :ambient, :diffuse, :specular)
end
function Base.getproperty(x::ModelLight, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :mode && return view(model.light_mode, index + 1, Base.OneTo(1))
    f === :bodyid && return view(model.light_bodyid, index + 1, Base.OneTo(1))
    f === :targetbodyid && return view(model.light_targetbodyid, index + 1, Base.OneTo(1))
    f === :directional && return view(model.light_directional, index + 1, Base.OneTo(1))
    f === :castshadow && return view(model.light_castshadow, index + 1, Base.OneTo(1))
    f === :active && return view(model.light_active, index + 1, Base.OneTo(1))
    f === :pos && return view(model.light_pos, index + 1, Base.OneTo(3))
    f === :dir && return view(model.light_dir, index + 1, Base.OneTo(3))
    f === :poscom0 && return view(model.light_poscom0, index + 1, Base.OneTo(3))
    f === :pos0 && return view(model.light_pos0, index + 1, Base.OneTo(3))
    f === :dir0 && return view(model.light_dir0, index + 1, Base.OneTo(3))
    f === :attenuation && return view(model.light_attenuation, index + 1, Base.OneTo(3))
    f === :cutoff && return view(model.light_cutoff, index + 1, Base.OneTo(1))
    f === :exponent && return view(model.light_exponent, index + 1, Base.OneTo(1))
    f === :ambient && return view(model.light_ambient, index + 1, Base.OneTo(3))
    f === :diffuse && return view(model.light_diffuse, index + 1, Base.OneTo(3))
    f === :specular && return view(model.light_specular, index + 1, Base.OneTo(3))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_LIGHT, index)
    error("Could not find the property: " * string(f))
end
function cam(model::Model, index::Integer)
    return ModelCamera(model, index)
end
function cam(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_CAMERA, name)
    return ModelCamera(model, index)
end
function cam(model::NamedModel, index::Integer)
    return ModelCamera(getfield(model, :model), index)
end
function cam(model::NamedModel, name::Symbol)
    index = index_by_name(model, :cam, name)
    return ModelCamera(getfield(model, :model), index)
end
function Base.propertynames(::ModelCamera)
    (:id, :name, :mode, :bodyid, :targetbodyid, :pos, :quat, :poscom0, :pos0, :mat0, :fovy, :ipd, :user)
end
function Base.getproperty(x::ModelCamera, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :mode && return view(model.cam_mode, index + 1, Base.OneTo(1))
    f === :bodyid && return view(model.cam_bodyid, index + 1, Base.OneTo(1))
    f === :targetbodyid && return view(model.cam_targetbodyid, index + 1, Base.OneTo(1))
    f === :pos && return view(model.cam_pos, index + 1, Base.OneTo(3))
    f === :quat && return view(model.cam_quat, index + 1, Base.OneTo(4))
    f === :poscom0 && return view(model.cam_poscom0, index + 1, Base.OneTo(3))
    f === :pos0 && return view(model.cam_pos0, index + 1, Base.OneTo(3))
    f === :mat0 && return view(model.cam_mat0, index + 1, Base.OneTo(9))
    f === :fovy && return view(model.cam_fovy, index + 1, Base.OneTo(1))
    f === :ipd && return view(model.cam_ipd, index + 1, Base.OneTo(1))
    f === :user && return view(model.cam_user, index + 1, Base.OneTo(model.nuser_cam))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_CAMERA, index)
    error("Could not find the property: " * string(f))
end
function tuple(model::Model, index::Integer)
    return ModelTuple(model, index)
end
function tuple(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_TUPLE, name)
    return ModelTuple(model, index)
end
function tuple(model::NamedModel, index::Integer)
    return ModelTuple(getfield(model, :model), index)
end
function tuple(model::NamedModel, name::Symbol)
    index = index_by_name(model, :tuple, name)
    return ModelTuple(getfield(model, :model), index)
end
function Base.propertynames(::ModelTuple)
    (:id, :name, :adr, :size)
end
function Base.getproperty(x::ModelTuple, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :adr && return view(model.tuple_adr, index + 1, Base.OneTo(1))
    f === :size && return view(model.tuple_size, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_TUPLE, index)
    error("Could not find the property: " * string(f))
end
function skin(model::Model, index::Integer)
    return ModelSkin(model, index)
end
function skin(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_SKIN, name)
    return ModelSkin(model, index)
end
function skin(model::NamedModel, index::Integer)
    return ModelSkin(getfield(model, :model), index)
end
function skin(model::NamedModel, name::Symbol)
    index = index_by_name(model, :skin, name)
    return ModelSkin(getfield(model, :model), index)
end
function Base.propertynames(::ModelSkin)
    (:id, :name, :matid, :rgba, :inflate, :vertadr, :vertnum, :texcoordadr, :faceadr, :facenum, :boneadr, :bonenum)
end
function Base.getproperty(x::ModelSkin, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :matid && return view(model.skin_matid, index + 1, Base.OneTo(1))
    f === :rgba && return view(model.skin_rgba, index + 1, Base.OneTo(4))
    f === :inflate && return view(model.skin_inflate, index + 1, Base.OneTo(1))
    f === :vertadr && return view(model.skin_vertadr, index + 1, Base.OneTo(1))
    f === :vertnum && return view(model.skin_vertnum, index + 1, Base.OneTo(1))
    f === :texcoordadr && return view(model.skin_texcoordadr, index + 1, Base.OneTo(1))
    f === :faceadr && return view(model.skin_faceadr, index + 1, Base.OneTo(1))
    f === :facenum && return view(model.skin_facenum, index + 1, Base.OneTo(1))
    f === :boneadr && return view(model.skin_boneadr, index + 1, Base.OneTo(1))
    f === :bonenum && return view(model.skin_bonenum, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_SKIN, index)
    error("Could not find the property: " * string(f))
end
function exclude(model::Model, index::Integer)
    return ModelExclude(model, index)
end
function exclude(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_EXCLUDE, name)
    return ModelExclude(model, index)
end
function exclude(model::NamedModel, index::Integer)
    return ModelExclude(getfield(model, :model), index)
end
function exclude(model::NamedModel, name::Symbol)
    index = index_by_name(model, :exclude, name)
    return ModelExclude(getfield(model, :model), index)
end
function Base.propertynames(::ModelExclude)
    (:id, :name, :signature)
end
function Base.getproperty(x::ModelExclude, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :signature && return view(model.exclude_signature, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_EXCLUDE, index)
    error("Could not find the property: " * string(f))
end
function mesh(model::Model, index::Integer)
    return ModelMesh(model, index)
end
function mesh(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_MESH, name)
    return ModelMesh(model, index)
end
function mesh(model::NamedModel, index::Integer)
    return ModelMesh(getfield(model, :model), index)
end
function mesh(model::NamedModel, name::Symbol)
    index = index_by_name(model, :mesh, name)
    return ModelMesh(getfield(model, :model), index)
end
function Base.propertynames(::ModelMesh)
    (:id, :name, :vertadr, :vertnum, :texcoordadr, :faceadr, :facenum, :graphadr)
end
function Base.getproperty(x::ModelMesh, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :vertadr && return view(model.mesh_vertadr, index + 1, Base.OneTo(1))
    f === :vertnum && return view(model.mesh_vertnum, index + 1, Base.OneTo(1))
    f === :texcoordadr && return view(model.mesh_texcoordadr, index + 1, Base.OneTo(1))
    f === :faceadr && return view(model.mesh_faceadr, index + 1, Base.OneTo(1))
    f === :facenum && return view(model.mesh_facenum, index + 1, Base.OneTo(1))
    f === :graphadr && return view(model.mesh_graphadr, index + 1, Base.OneTo(1))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_MESH, index)
    error("Could not find the property: " * string(f))
end
function body(model::Model, index::Integer)
    return ModelBody(model, index)
end
function body(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_BODY, name)
    return ModelBody(model, index)
end
function body(model::NamedModel, index::Integer)
    return ModelBody(getfield(model, :model), index)
end
function body(model::NamedModel, name::Symbol)
    index = index_by_name(model, :body, name)
    return ModelBody(getfield(model, :model), index)
end
function Base.propertynames(::ModelBody)
    (:id, :name, :parentid, :rootid, :weldid, :mocapid, :jntnum, :jntadr, :dofnum, :dofadr, :geomnum, :geomadr, :simple, :sameframe, :pos, :quat, :ipos, :iquat, :mass, :subtreemass, :inertia, :invweight0, :user)
end
function Base.getproperty(x::ModelBody, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :parentid && return view(model.body_parentid, index + 1, Base.OneTo(1))
    f === :rootid && return view(model.body_rootid, index + 1, Base.OneTo(1))
    f === :weldid && return view(model.body_weldid, index + 1, Base.OneTo(1))
    f === :mocapid && return view(model.body_mocapid, index + 1, Base.OneTo(1))
    f === :jntnum && return view(model.body_jntnum, index + 1, Base.OneTo(1))
    f === :jntadr && return view(model.body_jntadr, index + 1, Base.OneTo(1))
    f === :dofnum && return view(model.body_dofnum, index + 1, Base.OneTo(1))
    f === :dofadr && return view(model.body_dofadr, index + 1, Base.OneTo(1))
    f === :geomnum && return view(model.body_geomnum, index + 1, Base.OneTo(1))
    f === :geomadr && return view(model.body_geomadr, index + 1, Base.OneTo(1))
    f === :simple && return view(model.body_simple, index + 1, Base.OneTo(1))
    f === :sameframe && return view(model.body_sameframe, index + 1, Base.OneTo(1))
    f === :pos && return view(model.body_pos, index + 1, Base.OneTo(3))
    f === :quat && return view(model.body_quat, index + 1, Base.OneTo(4))
    f === :ipos && return view(model.body_ipos, index + 1, Base.OneTo(3))
    f === :iquat && return view(model.body_iquat, index + 1, Base.OneTo(4))
    f === :mass && return view(model.body_mass, index + 1, Base.OneTo(1))
    f === :subtreemass && return view(model.body_subtreemass, index + 1, Base.OneTo(1))
    f === :inertia && return view(model.body_inertia, index + 1, Base.OneTo(3))
    f === :invweight0 && return view(model.body_invweight0, index + 1, Base.OneTo(2))
    f === :user && return view(model.body_user, index + 1, Base.OneTo(model.nuser_body))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_BODY, index)
    error("Could not find the property: " * string(f))
end
function jnt(model::Model, index::Integer)
    return ModelJoint(model, index)
end
function jnt(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_JOINT, name)
    return ModelJoint(model, index)
end
function jnt(model::NamedModel, index::Integer)
    return ModelJoint(getfield(model, :model), index)
end
function jnt(model::NamedModel, name::Symbol)
    index = index_by_name(model, :jnt, name)
    return ModelJoint(getfield(model, :model), index)
end
function Base.propertynames(::ModelJoint)
    (:id, :name, :type, :qposadr, :dofadr, :group, :limited, :pos, :axis, :stiffness, :range, :margin, :user)
end
function Base.getproperty(x::ModelJoint, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :type && return view(model.jnt_type, index + 1, Base.OneTo(1))
    f === :qposadr && return view(model.jnt_qposadr, index + 1, Base.OneTo(1))
    f === :dofadr && return view(model.jnt_dofadr, index + 1, Base.OneTo(1))
    f === :group && return view(model.jnt_group, index + 1, Base.OneTo(1))
    f === :limited && return view(model.jnt_limited, index + 1, Base.OneTo(1))
    f === :pos && return view(model.jnt_pos, index + 1, Base.OneTo(3))
    f === :axis && return view(model.jnt_axis, index + 1, Base.OneTo(3))
    f === :stiffness && return view(model.jnt_stiffness, index + 1, Base.OneTo(1))
    f === :range && return view(model.jnt_range, index + 1, Base.OneTo(2))
    f === :margin && return view(model.jnt_margin, index + 1, Base.OneTo(1))
    f === :user && return view(model.jnt_user, index + 1, Base.OneTo(model.nuser_jnt))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_JOINT, index)
    error("Could not find the property: " * string(f))
end
function tendon(model::Model, index::Integer)
    return ModelTendon(model, index)
end
function tendon(model::Model, name::String)
    index = index_by_name(model, LibMuJoCo.mjOBJ_TENDON, name)
    return ModelTendon(model, index)
end
function tendon(model::NamedModel, index::Integer)
    return ModelTendon(getfield(model, :model), index)
end
function tendon(model::NamedModel, name::Symbol)
    index = index_by_name(model, :tendon, name)
    return ModelTendon(getfield(model, :model), index)
end
function Base.propertynames(::ModelTendon)
    (:id, :name, :adr, :num, :matid, :group, :limited, :width, :solref_lim, :solimp_lim, :solref_fri, :solimp_fri, :range, :margin, :stiffness, :damping, :frictionloss, :lengthspring, :length0, :invweight0, :user, :rgba)
end
function Base.getproperty(x::ModelTendon, f::Symbol)
    model = getfield(x, :model)
    index = getfield(x, :index)
    f === :adr && return view(model.tendon_adr, index + 1, Base.OneTo(1))
    f === :num && return view(model.tendon_num, index + 1, Base.OneTo(1))
    f === :matid && return view(model.tendon_matid, index + 1, Base.OneTo(1))
    f === :group && return view(model.tendon_group, index + 1, Base.OneTo(1))
    f === :limited && return view(model.tendon_limited, index + 1, Base.OneTo(1))
    f === :width && return view(model.tendon_width, index + 1, Base.OneTo(1))
    f === :solref_lim && return begin
                size_arr = Int(LibMuJoCo.mjNREF)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.tendon_solref_lim + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :solimp_lim && return begin
                size_arr = Int(LibMuJoCo.mjNIMP)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.tendon_solimp_lim + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :solref_fri && return begin
                size_arr = Int(LibMuJoCo.mjNREF)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.tendon_solref_fri + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :solimp_fri && return begin
                size_arr = Int(LibMuJoCo.mjNIMP)
                offset = index * size_arr * sizeof(Float64)
                arr_pointer = Ptr{Float64}(model.tendon_solimp_fri + offset)
                UnsafeArray(arr_pointer, (size_arr,))
            end
    f === :range && return view(model.tendon_range, index + 1, Base.OneTo(2))
    f === :margin && return view(model.tendon_margin, index + 1, Base.OneTo(1))
    f === :stiffness && return view(model.tendon_stiffness, index + 1, Base.OneTo(1))
    f === :damping && return view(model.tendon_damping, index + 1, Base.OneTo(1))
    f === :frictionloss && return view(model.tendon_frictionloss, index + 1, Base.OneTo(1))
    f === :lengthspring && return view(model.tendon_lengthspring, index + 1, Base.OneTo(1))
    f === :length0 && return view(model.tendon_length0, index + 1, Base.OneTo(1))
    f === :invweight0 && return view(model.tendon_invweight0, index + 1, Base.OneTo(1))
    f === :user && return view(model.tendon_user, index + 1, Base.OneTo(model.nuser_tendon))
    f === :rgba && return view(model.tendon_rgba, index + 1, Base.OneTo(4))
    f === :id && return index
    f === :name && return name_by_index(model, LibMuJoCo.mjOBJ_TENDON, index)
    error("Could not find the property: " * string(f))
end
const camera = cam
const ten = tendon
const joint = jnt
const camera = cam
const equality = eq
const texture = tex
const keyframe = key
const material = mat
const joint = jnt
end
