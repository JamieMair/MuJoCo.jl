using UnsafeArrays
export Options, Statistics, Model, Data
struct Options
    internal_pointer::Ptr{mjOption}
end
struct Statistics
    internal_pointer::Ptr{mjStatistic}
end
mutable struct Model
    internal_pointer::Ptr{mjModel}
    function Model(internal_pointer::Ptr{mjModel})
        __model = new(internal_pointer)
        function __finalizer(__model)
            mj_deleteModel(__model.internal_pointer)
        end
        Base.finalizer(__finalizer, __model)
        return __model
    end
end
mutable struct Data
    internal_pointer::Ptr{mjData}
    model::Model
    function Data(internal_pointer::Ptr{mjData}, model::Model)
        __data = new(internal_pointer, model)
        function __finalizer(__data)
            mj_deleteData(__data.internal_pointer)
        end
        Base.finalizer(__finalizer, __data)
        return __data
    end
end
function Base.propertynames(x::Options)
    (:timestep, :apirate, :impratio, :tolerance, :noslip_tolerance, :mpr_tolerance, :gravity, :wind, :magnetic, :density, :viscosity, :o_margin, :o_solref, :o_solimp, :integrator, :collision, :cone, :jacobian, :solver, :iterations, :noslip_iterations, :mpr_iterations, :disableflags, :enableflags)
end
function Base.getproperty(x::Options, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :timestep && return unsafe_load(Ptr{Float64}(internal_pointer + 0))
    f === :apirate && return unsafe_load(Ptr{Float64}(internal_pointer + 8))
    f === :impratio && return unsafe_load(Ptr{Float64}(internal_pointer + 16))
    f === :tolerance && return unsafe_load(Ptr{Float64}(internal_pointer + 24))
    f === :noslip_tolerance && return unsafe_load(Ptr{Float64}(internal_pointer + 32))
    f === :mpr_tolerance && return unsafe_load(Ptr{Float64}(internal_pointer + 40))
    f === :gravity && return UnsafeArray(Ptr{Float64}(internal_pointer + 48), (3,))
    f === :wind && return UnsafeArray(Ptr{Float64}(internal_pointer + 72), (3,))
    f === :magnetic && return UnsafeArray(Ptr{Float64}(internal_pointer + 96), (3,))
    f === :density && return unsafe_load(Ptr{Float64}(internal_pointer + 120))
    f === :viscosity && return unsafe_load(Ptr{Float64}(internal_pointer + 128))
    f === :o_margin && return unsafe_load(Ptr{Float64}(internal_pointer + 136))
    f === :o_solref && return UnsafeArray(Ptr{Float64}(internal_pointer + 144), (2,))
    f === :o_solimp && return UnsafeArray(Ptr{Float64}(internal_pointer + 160), (5,))
    f === :integrator && return unsafe_load(Ptr{Int32}(internal_pointer + 200))
    f === :collision && return unsafe_load(Ptr{Int32}(internal_pointer + 204))
    f === :cone && return unsafe_load(Ptr{Int32}(internal_pointer + 208))
    f === :jacobian && return unsafe_load(Ptr{Int32}(internal_pointer + 212))
    f === :solver && return unsafe_load(Ptr{Int32}(internal_pointer + 216))
    f === :iterations && return unsafe_load(Ptr{Int32}(internal_pointer + 220))
    f === :noslip_iterations && return unsafe_load(Ptr{Int32}(internal_pointer + 224))
    f === :mpr_iterations && return unsafe_load(Ptr{Int32}(internal_pointer + 228))
    f === :disableflags && return unsafe_load(Ptr{Int32}(internal_pointer + 232))
    f === :enableflags && return unsafe_load(Ptr{Int32}(internal_pointer + 236))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::Options, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :timestep
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :apirate
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :impratio
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 16), cvalue)
        return cvalue
    end
    if f === :tolerance
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 24), cvalue)
        return cvalue
    end
    if f === :noslip_tolerance
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 32), cvalue)
        return cvalue
    end
    if f === :mpr_tolerance
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 40), cvalue)
        return cvalue
    end
    if f === :density
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 48), cvalue)
        return cvalue
    end
    if f === :viscosity
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 56), cvalue)
        return cvalue
    end
    if f === :o_margin
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 64), cvalue)
        return cvalue
    end
    if f === :integrator
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 72), cvalue)
        return cvalue
    end
    if f === :collision
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 76), cvalue)
        return cvalue
    end
    if f === :cone
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 80), cvalue)
        return cvalue
    end
    if f === :jacobian
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 84), cvalue)
        return cvalue
    end
    if f === :solver
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 88), cvalue)
        return cvalue
    end
    if f === :iterations
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 92), cvalue)
        return cvalue
    end
    if f === :noslip_iterations
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 96), cvalue)
        return cvalue
    end
    if f === :mpr_iterations
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 100), cvalue)
        return cvalue
    end
    if f === :disableflags
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 104), cvalue)
        return cvalue
    end
    if f === :enableflags
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 108), cvalue)
        return cvalue
    end
    if f in (:gravity, :wind, :magnetic, :o_solref, :o_solimp)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjOption}}, wrapper::Options)
    return wrapper.internal_pointer
end
function Base.propertynames(x::Statistics)
    (:meaninertia, :meanmass, :meansize, :extent, :center)
end
function Base.getproperty(x::Statistics, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :meaninertia && return unsafe_load(Ptr{Float64}(internal_pointer + 0))
    f === :meanmass && return unsafe_load(Ptr{Float64}(internal_pointer + 8))
    f === :meansize && return unsafe_load(Ptr{Float64}(internal_pointer + 16))
    f === :extent && return unsafe_load(Ptr{Float64}(internal_pointer + 24))
    f === :center && return UnsafeArray(Ptr{Float64}(internal_pointer + 32), (3,))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::Statistics, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :meaninertia
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :meanmass
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :meansize
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 16), cvalue)
        return cvalue
    end
    if f === :extent
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 24), cvalue)
        return cvalue
    end
    if f in (:center,)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjStatistic}}, wrapper::Statistics)
    return wrapper.internal_pointer
end
function Base.propertynames(x::Model)
    (:nq, :nv, :nu, :na, :nbody, :nbvh, :njnt, :ngeom, :nsite, :ncam, :nlight, :nmesh, :nmeshvert, :nmeshnormal, :nmeshtexcoord, :nmeshface, :nmeshgraph, :nskin, :nskinvert, :nskintexvert, :nskinface, :nskinbone, :nskinbonevert, :nhfield, :nhfielddata, :ntex, :ntexdata, :nmat, :npair, :nexclude, :neq, :ntendon, :nwrap, :nsensor, :nnumeric, :nnumericdata, :ntext, :ntextdata, :ntuple, :ntupledata, :nkey, :nmocap, :nplugin, :npluginattr, :nuser_body, :nuser_jnt, :nuser_geom, :nuser_site, :nuser_cam, :nuser_tendon, :nuser_actuator, :nuser_sensor, :nnames, :nnames_map, :nM, :nD, :nB, :nemax, :njmax, :nconmax, :nstack, :nuserdata, :nsensordata, :npluginstate, :nbuffer, :opt, :vis, :stat, :buffer, :qpos0, :qpos_spring, :body_parentid, :body_rootid, :body_weldid, :body_mocapid, :body_jntnum, :body_jntadr, :body_dofnum, :body_dofadr, :body_geomnum, :body_geomadr, :body_simple, :body_sameframe, :body_pos, :body_quat, :body_ipos, :body_iquat, :body_mass, :body_subtreemass, :body_inertia, :body_invweight0, :body_gravcomp, :body_user, :body_plugin, :body_bvhadr, :body_bvhnum, :bvh_depth, :bvh_child, :bvh_geomid, :bvh_aabb, :jnt_type, :jnt_qposadr, :jnt_dofadr, :jnt_bodyid, :jnt_group, :jnt_limited, :jnt_actfrclimited, :jnt_solref, :jnt_solimp, :jnt_pos, :jnt_axis, :jnt_stiffness, :jnt_range, :jnt_actfrcrange, :jnt_margin, :jnt_user, :dof_bodyid, :dof_jntid, :dof_parentid, :dof_Madr, :dof_simplenum, :dof_solref, :dof_solimp, :dof_frictionloss, :dof_armature, :dof_damping, :dof_invweight0, :dof_M0, :geom_type, :geom_contype, :geom_conaffinity, :geom_condim, :geom_bodyid, :geom_dataid, :geom_matid, :geom_group, :geom_priority, :geom_sameframe, :geom_solmix, :geom_solref, :geom_solimp, :geom_size, :geom_aabb, :geom_rbound, :geom_pos, :geom_quat, :geom_friction, :geom_margin, :geom_gap, :geom_fluid, :geom_user, :geom_rgba, :site_type, :site_bodyid, :site_matid, :site_group, :site_sameframe, :site_size, :site_pos, :site_quat, :site_user, :site_rgba, :cam_mode, :cam_bodyid, :cam_targetbodyid, :cam_pos, :cam_quat, :cam_poscom0, :cam_pos0, :cam_mat0, :cam_fovy, :cam_ipd, :cam_user, :light_mode, :light_bodyid, :light_targetbodyid, :light_directional, :light_castshadow, :light_active, :light_pos, :light_dir, :light_poscom0, :light_pos0, :light_dir0, :light_attenuation, :light_cutoff, :light_exponent, :light_ambient, :light_diffuse, :light_specular, :mesh_vertadr, :mesh_vertnum, :mesh_faceadr, :mesh_facenum, :mesh_bvhadr, :mesh_bvhnum, :mesh_normaladr, :mesh_normalnum, :mesh_texcoordadr, :mesh_texcoordnum, :mesh_graphadr, :mesh_vert, :mesh_normal, :mesh_texcoord, :mesh_face, :mesh_facenormal, :mesh_facetexcoord, :mesh_graph, :skin_matid, :skin_group, :skin_rgba, :skin_inflate, :skin_vertadr, :skin_vertnum, :skin_texcoordadr, :skin_faceadr, :skin_facenum, :skin_boneadr, :skin_bonenum, :skin_vert, :skin_texcoord, :skin_face, :skin_bonevertadr, :skin_bonevertnum, :skin_bonebindpos, :skin_bonebindquat, :skin_bonebodyid, :skin_bonevertid, :skin_bonevertweight, :hfield_size, :hfield_nrow, :hfield_ncol, :hfield_adr, :hfield_data, :tex_type, :tex_height, :tex_width, :tex_adr, :tex_rgb, :mat_texid, :mat_texuniform, :mat_texrepeat, :mat_emission, :mat_specular, :mat_shininess, :mat_reflectance, :mat_rgba, :pair_dim, :pair_geom1, :pair_geom2, :pair_signature, :pair_solref, :pair_solreffriction, :pair_solimp, :pair_margin, :pair_gap, :pair_friction, :exclude_signature, :eq_type, :eq_obj1id, :eq_obj2id, :eq_active, :eq_solref, :eq_solimp, :eq_data, :tendon_adr, :tendon_num, :tendon_matid, :tendon_group, :tendon_limited, :tendon_width, :tendon_solref_lim, :tendon_solimp_lim, :tendon_solref_fri, :tendon_solimp_fri, :tendon_range, :tendon_margin, :tendon_stiffness, :tendon_damping, :tendon_frictionloss, :tendon_lengthspring, :tendon_length0, :tendon_invweight0, :tendon_user, :tendon_rgba, :wrap_type, :wrap_objid, :wrap_prm, :actuator_trntype, :actuator_dyntype, :actuator_gaintype, :actuator_biastype, :actuator_trnid, :actuator_actadr, :actuator_actnum, :actuator_group, :actuator_ctrllimited, :actuator_forcelimited, :actuator_actlimited, :actuator_dynprm, :actuator_gainprm, :actuator_biasprm, :actuator_ctrlrange, :actuator_forcerange, :actuator_actrange, :actuator_gear, :actuator_cranklength, :actuator_acc0, :actuator_length0, :actuator_lengthrange, :actuator_user, :actuator_plugin, :sensor_type, :sensor_datatype, :sensor_needstage, :sensor_objtype, :sensor_objid, :sensor_reftype, :sensor_refid, :sensor_dim, :sensor_adr, :sensor_cutoff, :sensor_noise, :sensor_user, :sensor_plugin, :plugin, :plugin_stateadr, :plugin_statenum, :plugin_attr, :plugin_attradr, :numeric_adr, :numeric_size, :numeric_data, :text_adr, :text_size, :text_data, :tuple_adr, :tuple_size, :tuple_objtype, :tuple_objid, :tuple_objprm, :key_time, :key_qpos, :key_qvel, :key_act, :key_mpos, :key_mquat, :key_ctrl, :name_bodyadr, :name_jntadr, :name_geomadr, :name_siteadr, :name_camadr, :name_lightadr, :name_meshadr, :name_skinadr, :name_hfieldadr, :name_texadr, :name_matadr, :name_pairadr, :name_excludeadr, :name_eqadr, :name_tendonadr, :name_actuatoradr, :name_sensoradr, :name_numericadr, :name_textadr, :name_tupleadr, :name_keyadr, :name_pluginadr, :names, :names_map)
end
function Base.getproperty(x::Model, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    model = x
    f === :internal_pointer && return internal_pointer
    f === :nq && return unsafe_load(Ptr{Int32}(internal_pointer + 0))
    f === :nv && return unsafe_load(Ptr{Int32}(internal_pointer + 4))
    f === :nu && return unsafe_load(Ptr{Int32}(internal_pointer + 8))
    f === :na && return unsafe_load(Ptr{Int32}(internal_pointer + 12))
    f === :nbody && return unsafe_load(Ptr{Int32}(internal_pointer + 16))
    f === :nbvh && return unsafe_load(Ptr{Int32}(internal_pointer + 20))
    f === :njnt && return unsafe_load(Ptr{Int32}(internal_pointer + 24))
    f === :ngeom && return unsafe_load(Ptr{Int32}(internal_pointer + 28))
    f === :nsite && return unsafe_load(Ptr{Int32}(internal_pointer + 32))
    f === :ncam && return unsafe_load(Ptr{Int32}(internal_pointer + 36))
    f === :nlight && return unsafe_load(Ptr{Int32}(internal_pointer + 40))
    f === :nmesh && return unsafe_load(Ptr{Int32}(internal_pointer + 44))
    f === :nmeshvert && return unsafe_load(Ptr{Int32}(internal_pointer + 48))
    f === :nmeshnormal && return unsafe_load(Ptr{Int32}(internal_pointer + 52))
    f === :nmeshtexcoord && return unsafe_load(Ptr{Int32}(internal_pointer + 56))
    f === :nmeshface && return unsafe_load(Ptr{Int32}(internal_pointer + 60))
    f === :nmeshgraph && return unsafe_load(Ptr{Int32}(internal_pointer + 64))
    f === :nskin && return unsafe_load(Ptr{Int32}(internal_pointer + 68))
    f === :nskinvert && return unsafe_load(Ptr{Int32}(internal_pointer + 72))
    f === :nskintexvert && return unsafe_load(Ptr{Int32}(internal_pointer + 76))
    f === :nskinface && return unsafe_load(Ptr{Int32}(internal_pointer + 80))
    f === :nskinbone && return unsafe_load(Ptr{Int32}(internal_pointer + 84))
    f === :nskinbonevert && return unsafe_load(Ptr{Int32}(internal_pointer + 88))
    f === :nhfield && return unsafe_load(Ptr{Int32}(internal_pointer + 92))
    f === :nhfielddata && return unsafe_load(Ptr{Int32}(internal_pointer + 96))
    f === :ntex && return unsafe_load(Ptr{Int32}(internal_pointer + 100))
    f === :ntexdata && return unsafe_load(Ptr{Int32}(internal_pointer + 104))
    f === :nmat && return unsafe_load(Ptr{Int32}(internal_pointer + 108))
    f === :npair && return unsafe_load(Ptr{Int32}(internal_pointer + 112))
    f === :nexclude && return unsafe_load(Ptr{Int32}(internal_pointer + 116))
    f === :neq && return unsafe_load(Ptr{Int32}(internal_pointer + 120))
    f === :ntendon && return unsafe_load(Ptr{Int32}(internal_pointer + 124))
    f === :nwrap && return unsafe_load(Ptr{Int32}(internal_pointer + 128))
    f === :nsensor && return unsafe_load(Ptr{Int32}(internal_pointer + 132))
    f === :nnumeric && return unsafe_load(Ptr{Int32}(internal_pointer + 136))
    f === :nnumericdata && return unsafe_load(Ptr{Int32}(internal_pointer + 140))
    f === :ntext && return unsafe_load(Ptr{Int32}(internal_pointer + 144))
    f === :ntextdata && return unsafe_load(Ptr{Int32}(internal_pointer + 148))
    f === :ntuple && return unsafe_load(Ptr{Int32}(internal_pointer + 152))
    f === :ntupledata && return unsafe_load(Ptr{Int32}(internal_pointer + 156))
    f === :nkey && return unsafe_load(Ptr{Int32}(internal_pointer + 160))
    f === :nmocap && return unsafe_load(Ptr{Int32}(internal_pointer + 164))
    f === :nplugin && return unsafe_load(Ptr{Int32}(internal_pointer + 168))
    f === :npluginattr && return unsafe_load(Ptr{Int32}(internal_pointer + 172))
    f === :nuser_body && return unsafe_load(Ptr{Int32}(internal_pointer + 176))
    f === :nuser_jnt && return unsafe_load(Ptr{Int32}(internal_pointer + 180))
    f === :nuser_geom && return unsafe_load(Ptr{Int32}(internal_pointer + 184))
    f === :nuser_site && return unsafe_load(Ptr{Int32}(internal_pointer + 188))
    f === :nuser_cam && return unsafe_load(Ptr{Int32}(internal_pointer + 192))
    f === :nuser_tendon && return unsafe_load(Ptr{Int32}(internal_pointer + 196))
    f === :nuser_actuator && return unsafe_load(Ptr{Int32}(internal_pointer + 200))
    f === :nuser_sensor && return unsafe_load(Ptr{Int32}(internal_pointer + 204))
    f === :nnames && return unsafe_load(Ptr{Int32}(internal_pointer + 208))
    f === :nnames_map && return unsafe_load(Ptr{Int32}(internal_pointer + 212))
    f === :nM && return unsafe_load(Ptr{Int32}(internal_pointer + 216))
    f === :nD && return unsafe_load(Ptr{Int32}(internal_pointer + 220))
    f === :nB && return unsafe_load(Ptr{Int32}(internal_pointer + 224))
    f === :nemax && return unsafe_load(Ptr{Int32}(internal_pointer + 228))
    f === :njmax && return unsafe_load(Ptr{Int32}(internal_pointer + 232))
    f === :nconmax && return unsafe_load(Ptr{Int32}(internal_pointer + 236))
    f === :nstack && return unsafe_load(Ptr{Int32}(internal_pointer + 240))
    f === :nuserdata && return unsafe_load(Ptr{Int32}(internal_pointer + 244))
    f === :nsensordata && return unsafe_load(Ptr{Int32}(internal_pointer + 248))
    f === :npluginstate && return unsafe_load(Ptr{Int32}(internal_pointer + 252))
    f === :nbuffer && return unsafe_load(Ptr{Int32}(internal_pointer + 256))
    f === :opt && return Options(Ptr{mjOption_}(internal_pointer + 264))
    f === :vis && return unsafe_load(Ptr{mjVisual_}(internal_pointer + 504))
    f === :stat && return Statistics(Ptr{mjStatistic_}(internal_pointer + 1072))
    f === :buffer && return unsafe_load(Ptr{Ptr{Nothing}}(internal_pointer + 1128))
    f === :qpos0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1136)), (Int(1), Int(x.nq))))
    f === :qpos_spring && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1144)), (Int(1), Int(x.nq))))
    f === :body_parentid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1152)), (Int(1), Int(x.nbody))))
    f === :body_rootid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1160)), (Int(1), Int(x.nbody))))
    f === :body_weldid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1168)), (Int(1), Int(x.nbody))))
    f === :body_mocapid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1176)), (Int(1), Int(x.nbody))))
    f === :body_jntnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1184)), (Int(1), Int(x.nbody))))
    f === :body_jntadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1192)), (Int(1), Int(x.nbody))))
    f === :body_dofnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1200)), (Int(1), Int(x.nbody))))
    f === :body_dofadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1208)), (Int(1), Int(x.nbody))))
    f === :body_geomnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1216)), (Int(1), Int(x.nbody))))
    f === :body_geomadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1224)), (Int(1), Int(x.nbody))))
    f === :body_simple && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1232)), (Int(1), Int(x.nbody))))
    f === :body_sameframe && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1240)), (Int(1), Int(x.nbody))))
    f === :body_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1248)), (Int(3), Int(x.nbody))))
    f === :body_quat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1256)), (Int(4), Int(x.nbody))))
    f === :body_ipos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1264)), (Int(3), Int(x.nbody))))
    f === :body_iquat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1272)), (Int(4), Int(x.nbody))))
    f === :body_mass && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1280)), (Int(1), Int(x.nbody))))
    f === :body_subtreemass && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1288)), (Int(1), Int(x.nbody))))
    f === :body_inertia && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1296)), (Int(3), Int(x.nbody))))
    f === :body_invweight0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1304)), (Int(2), Int(x.nbody))))
    f === :body_gravcomp && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1312)), (Int(1), Int(x.nbody))))
    f === :body_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1320)), (Int(model.nuser_body), Int(x.nbody))))
    f === :body_plugin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1328)), (Int(1), Int(x.nbody))))
    f === :body_bvhadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1336)), (Int(1), Int(x.nbody))))
    f === :body_bvhnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1344)), (Int(1), Int(x.nbody))))
    f === :bvh_depth && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1352)), (Int(1), Int(x.nbvh))))
    f === :bvh_child && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1360)), (Int(2), Int(x.nbvh))))
    f === :bvh_geomid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1368)), (Int(1), Int(x.nbvh))))
    f === :bvh_aabb && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1376)), (Int(6), Int(x.nbvh))))
    f === :jnt_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1384)), (Int(1), Int(x.njnt))))
    f === :jnt_qposadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1392)), (Int(1), Int(x.njnt))))
    f === :jnt_dofadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1400)), (Int(1), Int(x.njnt))))
    f === :jnt_bodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1408)), (Int(1), Int(x.njnt))))
    f === :jnt_group && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1416)), (Int(1), Int(x.njnt))))
    f === :jnt_limited && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1424)), (Int(1), Int(x.njnt))))
    f === :jnt_actfrclimited && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1432)), (Int(1), Int(x.njnt))))
    f === :jnt_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1440))
    f === :jnt_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1448))
    f === :jnt_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1456)), (Int(3), Int(x.njnt))))
    f === :jnt_axis && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1464)), (Int(3), Int(x.njnt))))
    f === :jnt_stiffness && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1472)), (Int(1), Int(x.njnt))))
    f === :jnt_range && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1480)), (Int(2), Int(x.njnt))))
    f === :jnt_actfrcrange && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1488)), (Int(2), Int(x.njnt))))
    f === :jnt_margin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1496)), (Int(1), Int(x.njnt))))
    f === :jnt_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1504)), (Int(model.nuser_jnt), Int(x.njnt))))
    f === :dof_bodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1512)), (Int(1), Int(x.nv))))
    f === :dof_jntid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1520)), (Int(1), Int(x.nv))))
    f === :dof_parentid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1528)), (Int(1), Int(x.nv))))
    f === :dof_Madr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1536)), (Int(1), Int(x.nv))))
    f === :dof_simplenum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1544)), (Int(1), Int(x.nv))))
    f === :dof_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1552))
    f === :dof_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1560))
    f === :dof_frictionloss && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1568)), (Int(1), Int(x.nv))))
    f === :dof_armature && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1576)), (Int(1), Int(x.nv))))
    f === :dof_damping && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1584)), (Int(1), Int(x.nv))))
    f === :dof_invweight0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1592)), (Int(1), Int(x.nv))))
    f === :dof_M0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1600)), (Int(1), Int(x.nv))))
    f === :geom_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1608)), (Int(1), Int(x.ngeom))))
    f === :geom_contype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1616)), (Int(1), Int(x.ngeom))))
    f === :geom_conaffinity && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1624)), (Int(1), Int(x.ngeom))))
    f === :geom_condim && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1632)), (Int(1), Int(x.ngeom))))
    f === :geom_bodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1640)), (Int(1), Int(x.ngeom))))
    f === :geom_dataid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1648)), (Int(1), Int(x.ngeom))))
    f === :geom_matid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1656)), (Int(1), Int(x.ngeom))))
    f === :geom_group && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1664)), (Int(1), Int(x.ngeom))))
    f === :geom_priority && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1672)), (Int(1), Int(x.ngeom))))
    f === :geom_sameframe && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1680)), (Int(1), Int(x.ngeom))))
    f === :geom_solmix && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1688)), (Int(1), Int(x.ngeom))))
    f === :geom_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1696))
    f === :geom_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1704))
    f === :geom_size && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1712)), (Int(3), Int(x.ngeom))))
    f === :geom_aabb && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1720)), (Int(6), Int(x.ngeom))))
    f === :geom_rbound && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1728)), (Int(1), Int(x.ngeom))))
    f === :geom_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1736)), (Int(3), Int(x.ngeom))))
    f === :geom_quat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1744)), (Int(4), Int(x.ngeom))))
    f === :geom_friction && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1752)), (Int(3), Int(x.ngeom))))
    f === :geom_margin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1760)), (Int(1), Int(x.ngeom))))
    f === :geom_gap && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1768)), (Int(1), Int(x.ngeom))))
    f === :geom_fluid && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1776))
    f === :geom_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1784)), (Int(model.nuser_geom), Int(x.ngeom))))
    f === :geom_rgba && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 1792)), (Int(4), Int(x.ngeom))))
    f === :site_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1800)), (Int(1), Int(x.nsite))))
    f === :site_bodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1808)), (Int(1), Int(x.nsite))))
    f === :site_matid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1816)), (Int(1), Int(x.nsite))))
    f === :site_group && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1824)), (Int(1), Int(x.nsite))))
    f === :site_sameframe && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1832)), (Int(1), Int(x.nsite))))
    f === :site_size && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1840)), (Int(3), Int(x.nsite))))
    f === :site_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1848)), (Int(3), Int(x.nsite))))
    f === :site_quat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1856)), (Int(4), Int(x.nsite))))
    f === :site_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1864)), (Int(model.nuser_site), Int(x.nsite))))
    f === :site_rgba && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 1872)), (Int(4), Int(x.nsite))))
    f === :cam_mode && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1880)), (Int(1), Int(x.ncam))))
    f === :cam_bodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1888)), (Int(1), Int(x.ncam))))
    f === :cam_targetbodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1896)), (Int(1), Int(x.ncam))))
    f === :cam_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1904)), (Int(3), Int(x.ncam))))
    f === :cam_quat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1912)), (Int(4), Int(x.ncam))))
    f === :cam_poscom0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1920)), (Int(3), Int(x.ncam))))
    f === :cam_pos0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1928)), (Int(3), Int(x.ncam))))
    f === :cam_mat0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1936)), (Int(9), Int(x.ncam))))
    f === :cam_fovy && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1944)), (Int(1), Int(x.ncam))))
    f === :cam_ipd && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1952)), (Int(1), Int(x.ncam))))
    f === :cam_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1960)), (Int(model.nuser_cam), Int(x.ncam))))
    f === :light_mode && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1968)), (Int(1), Int(x.nlight))))
    f === :light_bodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1976)), (Int(1), Int(x.nlight))))
    f === :light_targetbodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1984)), (Int(1), Int(x.nlight))))
    f === :light_directional && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1992)), (Int(1), Int(x.nlight))))
    f === :light_castshadow && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2000)), (Int(1), Int(x.nlight))))
    f === :light_active && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2008)), (Int(1), Int(x.nlight))))
    f === :light_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2016)), (Int(3), Int(x.nlight))))
    f === :light_dir && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2024)), (Int(3), Int(x.nlight))))
    f === :light_poscom0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2032)), (Int(3), Int(x.nlight))))
    f === :light_pos0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2040)), (Int(3), Int(x.nlight))))
    f === :light_dir0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2048)), (Int(3), Int(x.nlight))))
    f === :light_attenuation && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2056)), (Int(3), Int(x.nlight))))
    f === :light_cutoff && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2064)), (Int(1), Int(x.nlight))))
    f === :light_exponent && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2072)), (Int(1), Int(x.nlight))))
    f === :light_ambient && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2080)), (Int(3), Int(x.nlight))))
    f === :light_diffuse && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2088)), (Int(3), Int(x.nlight))))
    f === :light_specular && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2096)), (Int(3), Int(x.nlight))))
    f === :mesh_vertadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2104)), (Int(1), Int(x.nmesh))))
    f === :mesh_vertnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2112)), (Int(1), Int(x.nmesh))))
    f === :mesh_faceadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2120)), (Int(1), Int(x.nmesh))))
    f === :mesh_facenum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2128)), (Int(1), Int(x.nmesh))))
    f === :mesh_bvhadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2136)), (Int(1), Int(x.nmesh))))
    f === :mesh_bvhnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2144)), (Int(1), Int(x.nmesh))))
    f === :mesh_normaladr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2152)), (Int(1), Int(x.nmesh))))
    f === :mesh_normalnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2160)), (Int(1), Int(x.nmesh))))
    f === :mesh_texcoordadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2168)), (Int(1), Int(x.nmesh))))
    f === :mesh_texcoordnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2176)), (Int(1), Int(x.nmesh))))
    f === :mesh_graphadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2184)), (Int(1), Int(x.nmesh))))
    f === :mesh_vert && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2192)), (Int(3), Int(x.nmeshvert))))
    f === :mesh_normal && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2200)), (Int(3), Int(x.nmeshnormal))))
    f === :mesh_texcoord && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2208)), (Int(2), Int(x.nmeshtexcoord))))
    f === :mesh_face && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2216)), (Int(3), Int(x.nmeshface))))
    f === :mesh_facenormal && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2224)), (Int(3), Int(x.nmeshface))))
    f === :mesh_facetexcoord && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2232)), (Int(3), Int(x.nmeshface))))
    f === :mesh_graph && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2240)), (Int(1), Int(x.nmeshgraph))))
    f === :skin_matid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2248)), (Int(1), Int(x.nskin))))
    f === :skin_group && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2256)), (Int(1), Int(x.nskin))))
    f === :skin_rgba && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2264)), (Int(4), Int(x.nskin))))
    f === :skin_inflate && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2272)), (Int(1), Int(x.nskin))))
    f === :skin_vertadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2280)), (Int(1), Int(x.nskin))))
    f === :skin_vertnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2288)), (Int(1), Int(x.nskin))))
    f === :skin_texcoordadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2296)), (Int(1), Int(x.nskin))))
    f === :skin_faceadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2304)), (Int(1), Int(x.nskin))))
    f === :skin_facenum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2312)), (Int(1), Int(x.nskin))))
    f === :skin_boneadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2320)), (Int(1), Int(x.nskin))))
    f === :skin_bonenum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2328)), (Int(1), Int(x.nskin))))
    f === :skin_vert && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2336)), (Int(3), Int(x.nskinvert))))
    f === :skin_texcoord && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2344)), (Int(2), Int(x.nskintexvert))))
    f === :skin_face && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2352)), (Int(3), Int(x.nskinface))))
    f === :skin_bonevertadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2360)), (Int(1), Int(x.nskinbone))))
    f === :skin_bonevertnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2368)), (Int(1), Int(x.nskinbone))))
    f === :skin_bonebindpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2376)), (Int(3), Int(x.nskinbone))))
    f === :skin_bonebindquat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2384)), (Int(4), Int(x.nskinbone))))
    f === :skin_bonebodyid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2392)), (Int(1), Int(x.nskinbone))))
    f === :skin_bonevertid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2400)), (Int(1), Int(x.nskinbonevert))))
    f === :skin_bonevertweight && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2408)), (Int(1), Int(x.nskinbonevert))))
    f === :hfield_size && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2416)), (Int(4), Int(x.nhfield))))
    f === :hfield_nrow && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2424)), (Int(1), Int(x.nhfield))))
    f === :hfield_ncol && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2432)), (Int(1), Int(x.nhfield))))
    f === :hfield_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2440)), (Int(1), Int(x.nhfield))))
    f === :hfield_data && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2448)), (Int(1), Int(x.nhfielddata))))
    f === :tex_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2456)), (Int(1), Int(x.ntex))))
    f === :tex_height && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2464)), (Int(1), Int(x.ntex))))
    f === :tex_width && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2472)), (Int(1), Int(x.ntex))))
    f === :tex_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2480)), (Int(1), Int(x.ntex))))
    f === :tex_rgb && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2488)), (Int(1), Int(x.ntexdata))))
    f === :mat_texid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2496)), (Int(1), Int(x.nmat))))
    f === :mat_texuniform && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2504)), (Int(1), Int(x.nmat))))
    f === :mat_texrepeat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2512)), (Int(2), Int(x.nmat))))
    f === :mat_emission && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2520)), (Int(1), Int(x.nmat))))
    f === :mat_specular && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2528)), (Int(1), Int(x.nmat))))
    f === :mat_shininess && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2536)), (Int(1), Int(x.nmat))))
    f === :mat_reflectance && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2544)), (Int(1), Int(x.nmat))))
    f === :mat_rgba && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2552)), (Int(4), Int(x.nmat))))
    f === :pair_dim && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2560)), (Int(1), Int(x.npair))))
    f === :pair_geom1 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2568)), (Int(1), Int(x.npair))))
    f === :pair_geom2 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2576)), (Int(1), Int(x.npair))))
    f === :pair_signature && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2584)), (Int(1), Int(x.npair))))
    f === :pair_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2592))
    f === :pair_solreffriction && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2600))
    f === :pair_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2608))
    f === :pair_margin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2616)), (Int(1), Int(x.npair))))
    f === :pair_gap && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2624)), (Int(1), Int(x.npair))))
    f === :pair_friction && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2632)), (Int(5), Int(x.npair))))
    f === :exclude_signature && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2640)), (Int(1), Int(x.nexclude))))
    f === :eq_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2648)), (Int(1), Int(x.neq))))
    f === :eq_obj1id && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2656)), (Int(1), Int(x.neq))))
    f === :eq_obj2id && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2664)), (Int(1), Int(x.neq))))
    f === :eq_active && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2672)), (Int(1), Int(x.neq))))
    f === :eq_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2680))
    f === :eq_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2688))
    f === :eq_data && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2696))
    f === :tendon_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2704)), (Int(1), Int(x.ntendon))))
    f === :tendon_num && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2712)), (Int(1), Int(x.ntendon))))
    f === :tendon_matid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2720)), (Int(1), Int(x.ntendon))))
    f === :tendon_group && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2728)), (Int(1), Int(x.ntendon))))
    f === :tendon_limited && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2736)), (Int(1), Int(x.ntendon))))
    f === :tendon_width && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2744)), (Int(1), Int(x.ntendon))))
    f === :tendon_solref_lim && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2752))
    f === :tendon_solimp_lim && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2760))
    f === :tendon_solref_fri && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2768))
    f === :tendon_solimp_fri && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2776))
    f === :tendon_range && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2784)), (Int(2), Int(x.ntendon))))
    f === :tendon_margin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2792)), (Int(1), Int(x.ntendon))))
    f === :tendon_stiffness && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2800)), (Int(1), Int(x.ntendon))))
    f === :tendon_damping && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2808)), (Int(1), Int(x.ntendon))))
    f === :tendon_frictionloss && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2816)), (Int(1), Int(x.ntendon))))
    f === :tendon_lengthspring && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2824)), (Int(2), Int(x.ntendon))))
    f === :tendon_length0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2832)), (Int(1), Int(x.ntendon))))
    f === :tendon_invweight0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2840)), (Int(1), Int(x.ntendon))))
    f === :tendon_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2848)), (Int(model.nuser_tendon), Int(x.ntendon))))
    f === :tendon_rgba && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2856)), (Int(4), Int(x.ntendon))))
    f === :wrap_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2864)), (Int(1), Int(x.nwrap))))
    f === :wrap_objid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2872)), (Int(1), Int(x.nwrap))))
    f === :wrap_prm && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2880)), (Int(1), Int(x.nwrap))))
    f === :actuator_trntype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2888)), (Int(1), Int(x.nu))))
    f === :actuator_dyntype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2896)), (Int(1), Int(x.nu))))
    f === :actuator_gaintype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2904)), (Int(1), Int(x.nu))))
    f === :actuator_biastype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2912)), (Int(1), Int(x.nu))))
    f === :actuator_trnid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2920)), (Int(2), Int(x.nu))))
    f === :actuator_actadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2928)), (Int(1), Int(x.nu))))
    f === :actuator_actnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2936)), (Int(1), Int(x.nu))))
    f === :actuator_group && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2944)), (Int(1), Int(x.nu))))
    f === :actuator_ctrllimited && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2952)), (Int(1), Int(x.nu))))
    f === :actuator_forcelimited && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2960)), (Int(1), Int(x.nu))))
    f === :actuator_actlimited && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2968)), (Int(1), Int(x.nu))))
    f === :actuator_dynprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2976))
    f === :actuator_gainprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2984))
    f === :actuator_biasprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2992))
    f === :actuator_ctrlrange && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3000)), (Int(2), Int(x.nu))))
    f === :actuator_forcerange && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3008)), (Int(2), Int(x.nu))))
    f === :actuator_actrange && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3016)), (Int(2), Int(x.nu))))
    f === :actuator_gear && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3024)), (Int(6), Int(x.nu))))
    f === :actuator_cranklength && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3032)), (Int(1), Int(x.nu))))
    f === :actuator_acc0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3040)), (Int(1), Int(x.nu))))
    f === :actuator_length0 && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3048)), (Int(1), Int(x.nu))))
    f === :actuator_lengthrange && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3056)), (Int(2), Int(x.nu))))
    f === :actuator_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3064)), (Int(model.nuser_actuator), Int(x.nu))))
    f === :actuator_plugin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3072)), (Int(1), Int(x.nu))))
    f === :sensor_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3080)), (Int(1), Int(x.nsensor))))
    f === :sensor_datatype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3088)), (Int(1), Int(x.nsensor))))
    f === :sensor_needstage && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3096)), (Int(1), Int(x.nsensor))))
    f === :sensor_objtype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3104)), (Int(1), Int(x.nsensor))))
    f === :sensor_objid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3112)), (Int(1), Int(x.nsensor))))
    f === :sensor_reftype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3120)), (Int(1), Int(x.nsensor))))
    f === :sensor_refid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3128)), (Int(1), Int(x.nsensor))))
    f === :sensor_dim && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3136)), (Int(1), Int(x.nsensor))))
    f === :sensor_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3144)), (Int(1), Int(x.nsensor))))
    f === :sensor_cutoff && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3152)), (Int(1), Int(x.nsensor))))
    f === :sensor_noise && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3160)), (Int(1), Int(x.nsensor))))
    f === :sensor_user && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3168)), (Int(model.nuser_sensor), Int(x.nsensor))))
    f === :sensor_plugin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3176)), (Int(1), Int(x.nsensor))))
    f === :plugin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3184)), (Int(1), Int(x.nplugin))))
    f === :plugin_stateadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3192)), (Int(1), Int(x.nplugin))))
    f === :plugin_statenum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3200)), (Int(1), Int(x.nplugin))))
    f === :plugin_attr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3208)), (Int(1), Int(x.npluginattr))))
    f === :plugin_attradr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3216)), (Int(1), Int(x.nplugin))))
    f === :numeric_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3224)), (Int(1), Int(x.nnumeric))))
    f === :numeric_size && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3232)), (Int(1), Int(x.nnumeric))))
    f === :numeric_data && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3240)), (Int(1), Int(x.nnumericdata))))
    f === :text_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3248)), (Int(1), Int(x.ntext))))
    f === :text_size && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3256)), (Int(1), Int(x.ntext))))
    f === :text_data && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3264)), (Int(1), Int(x.ntextdata))))
    f === :tuple_adr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3272)), (Int(1), Int(x.ntuple))))
    f === :tuple_size && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3280)), (Int(1), Int(x.ntuple))))
    f === :tuple_objtype && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3288)), (Int(1), Int(x.ntupledata))))
    f === :tuple_objid && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3296)), (Int(1), Int(x.ntupledata))))
    f === :tuple_objprm && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3304)), (Int(1), Int(x.ntupledata))))
    f === :key_time && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3312)), (Int(1), Int(x.nkey))))
    f === :key_qpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3320)), (Int(model.nq), Int(x.nkey))))
    f === :key_qvel && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3328)), (Int(model.nv), Int(x.nkey))))
    f === :key_act && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3336)), (Int(model.na), Int(x.nkey))))
    f === :key_mpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3344)), (Int(model.nmocap * 3), Int(x.nkey))))
    f === :key_mquat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3352)), (Int(model.nmocap * 4), Int(x.nkey))))
    f === :key_ctrl && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3360)), (Int(model.nu), Int(x.nkey))))
    f === :name_bodyadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3368)), (Int(1), Int(x.nbody))))
    f === :name_jntadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3376)), (Int(1), Int(x.njnt))))
    f === :name_geomadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3384)), (Int(1), Int(x.ngeom))))
    f === :name_siteadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3392)), (Int(1), Int(x.nsite))))
    f === :name_camadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3400)), (Int(1), Int(x.ncam))))
    f === :name_lightadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3408)), (Int(1), Int(x.nlight))))
    f === :name_meshadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3416)), (Int(1), Int(x.nmesh))))
    f === :name_skinadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3424)), (Int(1), Int(x.nskin))))
    f === :name_hfieldadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3432)), (Int(1), Int(x.nhfield))))
    f === :name_texadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3440)), (Int(1), Int(x.ntex))))
    f === :name_matadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3448)), (Int(1), Int(x.nmat))))
    f === :name_pairadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3456)), (Int(1), Int(x.npair))))
    f === :name_excludeadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3464)), (Int(1), Int(x.nexclude))))
    f === :name_eqadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3472)), (Int(1), Int(x.neq))))
    f === :name_tendonadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3480)), (Int(1), Int(x.ntendon))))
    f === :name_actuatoradr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3488)), (Int(1), Int(x.nu))))
    f === :name_sensoradr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3496)), (Int(1), Int(x.nsensor))))
    f === :name_numericadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3504)), (Int(1), Int(x.nnumeric))))
    f === :name_textadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3512)), (Int(1), Int(x.ntext))))
    f === :name_tupleadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3520)), (Int(1), Int(x.ntuple))))
    f === :name_keyadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3528)), (Int(1), Int(x.nkey))))
    f === :name_pluginadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3536)), (Int(1), Int(x.nplugin))))
    f === :names && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3544)), (Int(1), Int(x.nnames))))
    f === :names_map && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3552)), (Int(1), Int(x.nnames_map))))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::Model, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :nq
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :nv
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :nu
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :na
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 12), cvalue)
        return cvalue
    end
    if f === :nbody
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 16), cvalue)
        return cvalue
    end
    if f === :nbvh
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 20), cvalue)
        return cvalue
    end
    if f === :njnt
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 24), cvalue)
        return cvalue
    end
    if f === :ngeom
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 28), cvalue)
        return cvalue
    end
    if f === :nsite
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 32), cvalue)
        return cvalue
    end
    if f === :ncam
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 36), cvalue)
        return cvalue
    end
    if f === :nlight
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 40), cvalue)
        return cvalue
    end
    if f === :nmesh
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 44), cvalue)
        return cvalue
    end
    if f === :nmeshvert
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 48), cvalue)
        return cvalue
    end
    if f === :nmeshnormal
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 52), cvalue)
        return cvalue
    end
    if f === :nmeshtexcoord
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 56), cvalue)
        return cvalue
    end
    if f === :nmeshface
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 60), cvalue)
        return cvalue
    end
    if f === :nmeshgraph
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 64), cvalue)
        return cvalue
    end
    if f === :nskin
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 68), cvalue)
        return cvalue
    end
    if f === :nskinvert
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 72), cvalue)
        return cvalue
    end
    if f === :nskintexvert
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 76), cvalue)
        return cvalue
    end
    if f === :nskinface
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 80), cvalue)
        return cvalue
    end
    if f === :nskinbone
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 84), cvalue)
        return cvalue
    end
    if f === :nskinbonevert
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 88), cvalue)
        return cvalue
    end
    if f === :nhfield
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 92), cvalue)
        return cvalue
    end
    if f === :nhfielddata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 96), cvalue)
        return cvalue
    end
    if f === :ntex
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 100), cvalue)
        return cvalue
    end
    if f === :ntexdata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 104), cvalue)
        return cvalue
    end
    if f === :nmat
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 108), cvalue)
        return cvalue
    end
    if f === :npair
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 112), cvalue)
        return cvalue
    end
    if f === :nexclude
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 116), cvalue)
        return cvalue
    end
    if f === :neq
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 120), cvalue)
        return cvalue
    end
    if f === :ntendon
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 124), cvalue)
        return cvalue
    end
    if f === :nwrap
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 128), cvalue)
        return cvalue
    end
    if f === :nsensor
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 132), cvalue)
        return cvalue
    end
    if f === :nnumeric
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 136), cvalue)
        return cvalue
    end
    if f === :nnumericdata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 140), cvalue)
        return cvalue
    end
    if f === :ntext
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 144), cvalue)
        return cvalue
    end
    if f === :ntextdata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 148), cvalue)
        return cvalue
    end
    if f === :ntuple
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 152), cvalue)
        return cvalue
    end
    if f === :ntupledata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 156), cvalue)
        return cvalue
    end
    if f === :nkey
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 160), cvalue)
        return cvalue
    end
    if f === :nmocap
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 164), cvalue)
        return cvalue
    end
    if f === :nplugin
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 168), cvalue)
        return cvalue
    end
    if f === :npluginattr
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 172), cvalue)
        return cvalue
    end
    if f === :nuser_body
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 176), cvalue)
        return cvalue
    end
    if f === :nuser_jnt
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 180), cvalue)
        return cvalue
    end
    if f === :nuser_geom
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 184), cvalue)
        return cvalue
    end
    if f === :nuser_site
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 188), cvalue)
        return cvalue
    end
    if f === :nuser_cam
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 192), cvalue)
        return cvalue
    end
    if f === :nuser_tendon
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 196), cvalue)
        return cvalue
    end
    if f === :nuser_actuator
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 200), cvalue)
        return cvalue
    end
    if f === :nuser_sensor
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 204), cvalue)
        return cvalue
    end
    if f === :nnames
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 208), cvalue)
        return cvalue
    end
    if f === :nnames_map
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 212), cvalue)
        return cvalue
    end
    if f === :nM
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 216), cvalue)
        return cvalue
    end
    if f === :nD
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 220), cvalue)
        return cvalue
    end
    if f === :nB
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 224), cvalue)
        return cvalue
    end
    if f === :nemax
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 228), cvalue)
        return cvalue
    end
    if f === :njmax
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 232), cvalue)
        return cvalue
    end
    if f === :nconmax
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 236), cvalue)
        return cvalue
    end
    if f === :nstack
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 240), cvalue)
        return cvalue
    end
    if f === :nuserdata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 244), cvalue)
        return cvalue
    end
    if f === :nsensordata
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 248), cvalue)
        return cvalue
    end
    if f === :npluginstate
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 252), cvalue)
        return cvalue
    end
    if f === :nbuffer
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 256), cvalue)
        return cvalue
    end
    if f === :opt
        cvalue = convert(mjOption_, value)
        unsafe_store!(Ptr{mjOption_}(internal_pointer + 260), cvalue)
        return cvalue
    end
    if f === :vis
        cvalue = convert(mjVisual_, value)
        unsafe_store!(Ptr{mjVisual_}(internal_pointer + 500), cvalue)
        return cvalue
    end
    if f === :stat
        cvalue = convert(mjStatistic_, value)
        unsafe_store!(Ptr{mjStatistic_}(internal_pointer + 1068), cvalue)
        return cvalue
    end
    if f in (:buffer, :qpos0, :qpos_spring, :body_parentid, :body_rootid, :body_weldid, :body_mocapid, :body_jntnum, :body_jntadr, :body_dofnum, :body_dofadr, :body_geomnum, :body_geomadr, :body_simple, :body_sameframe, :body_pos, :body_quat, :body_ipos, :body_iquat, :body_mass, :body_subtreemass, :body_inertia, :body_invweight0, :body_gravcomp, :body_user, :body_plugin, :body_bvhadr, :body_bvhnum, :bvh_depth, :bvh_child, :bvh_geomid, :bvh_aabb, :jnt_type, :jnt_qposadr, :jnt_dofadr, :jnt_bodyid, :jnt_group, :jnt_limited, :jnt_actfrclimited, :jnt_solref, :jnt_solimp, :jnt_pos, :jnt_axis, :jnt_stiffness, :jnt_range, :jnt_actfrcrange, :jnt_margin, :jnt_user, :dof_bodyid, :dof_jntid, :dof_parentid, :dof_Madr, :dof_simplenum, :dof_solref, :dof_solimp, :dof_frictionloss, :dof_armature, :dof_damping, :dof_invweight0, :dof_M0, :geom_type, :geom_contype, :geom_conaffinity, :geom_condim, :geom_bodyid, :geom_dataid, :geom_matid, :geom_group, :geom_priority, :geom_sameframe, :geom_solmix, :geom_solref, :geom_solimp, :geom_size, :geom_aabb, :geom_rbound, :geom_pos, :geom_quat, :geom_friction, :geom_margin, :geom_gap, :geom_fluid, :geom_user, :geom_rgba, :site_type, :site_bodyid, :site_matid, :site_group, :site_sameframe, :site_size, :site_pos, :site_quat, :site_user, :site_rgba, :cam_mode, :cam_bodyid, :cam_targetbodyid, :cam_pos, :cam_quat, :cam_poscom0, :cam_pos0, :cam_mat0, :cam_fovy, :cam_ipd, :cam_user, :light_mode, :light_bodyid, :light_targetbodyid, :light_directional, :light_castshadow, :light_active, :light_pos, :light_dir, :light_poscom0, :light_pos0, :light_dir0, :light_attenuation, :light_cutoff, :light_exponent, :light_ambient, :light_diffuse, :light_specular, :mesh_vertadr, :mesh_vertnum, :mesh_faceadr, :mesh_facenum, :mesh_bvhadr, :mesh_bvhnum, :mesh_normaladr, :mesh_normalnum, :mesh_texcoordadr, :mesh_texcoordnum, :mesh_graphadr, :mesh_vert, :mesh_normal, :mesh_texcoord, :mesh_face, :mesh_facenormal, :mesh_facetexcoord, :mesh_graph, :skin_matid, :skin_group, :skin_rgba, :skin_inflate, :skin_vertadr, :skin_vertnum, :skin_texcoordadr, :skin_faceadr, :skin_facenum, :skin_boneadr, :skin_bonenum, :skin_vert, :skin_texcoord, :skin_face, :skin_bonevertadr, :skin_bonevertnum, :skin_bonebindpos, :skin_bonebindquat, :skin_bonebodyid, :skin_bonevertid, :skin_bonevertweight, :hfield_size, :hfield_nrow, :hfield_ncol, :hfield_adr, :hfield_data, :tex_type, :tex_height, :tex_width, :tex_adr, :tex_rgb, :mat_texid, :mat_texuniform, :mat_texrepeat, :mat_emission, :mat_specular, :mat_shininess, :mat_reflectance, :mat_rgba, :pair_dim, :pair_geom1, :pair_geom2, :pair_signature, :pair_solref, :pair_solreffriction, :pair_solimp, :pair_margin, :pair_gap, :pair_friction, :exclude_signature, :eq_type, :eq_obj1id, :eq_obj2id, :eq_active, :eq_solref, :eq_solimp, :eq_data, :tendon_adr, :tendon_num, :tendon_matid, :tendon_group, :tendon_limited, :tendon_width, :tendon_solref_lim, :tendon_solimp_lim, :tendon_solref_fri, :tendon_solimp_fri, :tendon_range, :tendon_margin, :tendon_stiffness, :tendon_damping, :tendon_frictionloss, :tendon_lengthspring, :tendon_length0, :tendon_invweight0, :tendon_user, :tendon_rgba, :wrap_type, :wrap_objid, :wrap_prm, :actuator_trntype, :actuator_dyntype, :actuator_gaintype, :actuator_biastype, :actuator_trnid, :actuator_actadr, :actuator_actnum, :actuator_group, :actuator_ctrllimited, :actuator_forcelimited, :actuator_actlimited, :actuator_dynprm, :actuator_gainprm, :actuator_biasprm, :actuator_ctrlrange, :actuator_forcerange, :actuator_actrange, :actuator_gear, :actuator_cranklength, :actuator_acc0, :actuator_length0, :actuator_lengthrange, :actuator_user, :actuator_plugin, :sensor_type, :sensor_datatype, :sensor_needstage, :sensor_objtype, :sensor_objid, :sensor_reftype, :sensor_refid, :sensor_dim, :sensor_adr, :sensor_cutoff, :sensor_noise, :sensor_user, :sensor_plugin, :plugin, :plugin_stateadr, :plugin_statenum, :plugin_attr, :plugin_attradr, :numeric_adr, :numeric_size, :numeric_data, :text_adr, :text_size, :text_data, :tuple_adr, :tuple_size, :tuple_objtype, :tuple_objid, :tuple_objprm, :key_time, :key_qpos, :key_qvel, :key_act, :key_mpos, :key_mquat, :key_ctrl, :name_bodyadr, :name_jntadr, :name_geomadr, :name_siteadr, :name_camadr, :name_lightadr, :name_meshadr, :name_skinadr, :name_hfieldadr, :name_texadr, :name_matadr, :name_pairadr, :name_excludeadr, :name_eqadr, :name_tendonadr, :name_actuatoradr, :name_sensoradr, :name_numericadr, :name_textadr, :name_tupleadr, :name_keyadr, :name_pluginadr, :names, :names_map)
        error("Cannot overwrite a pointer field.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjModel}}, wrapper::Model)
    return wrapper.internal_pointer
end
function Base.propertynames(x::Data)
    (:nstack, :nbuffer, :nplugin, :pstack, :parena, :maxuse_stack, :maxuse_arena, :maxuse_con, :maxuse_efc, :warning, :timer, :solver, :solver_iter, :solver_nnz, :solver_fwdinv, :nbodypair_broad, :nbodypair_narrow, :ngeompair_mid, :ngeompair_narrow, :ne, :nf, :nefc, :nnzJ, :ncon, :time, :energy, :buffer, :arena, :qpos, :qvel, :act, :qacc_warmstart, :plugin_state, :ctrl, :qfrc_applied, :xfrc_applied, :mocap_pos, :mocap_quat, :qacc, :act_dot, :userdata, :sensordata, :plugin, :plugin_data, :xpos, :xquat, :xmat, :xipos, :ximat, :xanchor, :xaxis, :geom_xpos, :geom_xmat, :site_xpos, :site_xmat, :cam_xpos, :cam_xmat, :light_xpos, :light_xdir, :subtree_com, :cdof, :cinert, :ten_wrapadr, :ten_wrapnum, :ten_J_rownnz, :ten_J_rowadr, :ten_J_colind, :ten_length, :ten_J, :wrap_obj, :wrap_xpos, :actuator_length, :actuator_moment, :crb, :qM, :qLD, :qLDiagInv, :qLDiagSqrtInv, :bvh_active, :ten_velocity, :actuator_velocity, :cvel, :cdof_dot, :qfrc_bias, :qfrc_passive, :efc_vel, :efc_aref, :subtree_linvel, :subtree_angmom, :qH, :qHDiagInv, :D_rownnz, :D_rowadr, :D_colind, :B_rownnz, :B_rowadr, :B_colind, :qDeriv, :qLU, :actuator_force, :qfrc_actuator, :qfrc_smooth, :qacc_smooth, :qfrc_constraint, :qfrc_inverse, :cacc, :cfrc_int, :cfrc_ext, :contact, :efc_type, :efc_id, :efc_J_rownnz, :efc_J_rowadr, :efc_J_rowsuper, :efc_J_colind, :efc_JT_rownnz, :efc_JT_rowadr, :efc_JT_rowsuper, :efc_JT_colind, :efc_J, :efc_JT, :efc_pos, :efc_margin, :efc_frictionloss, :efc_diagApprox, :efc_KBIP, :efc_D, :efc_R, :efc_b, :efc_force, :efc_state, :efc_AR_rownnz, :efc_AR_rowadr, :efc_AR_colind, :efc_AR)
end
function Base.getproperty(x::Data, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    model = getfield(x, :model)
    data = x
    f === :internal_pointer && return internal_pointer
    f === :nstack && return unsafe_load(Ptr{Int32}(internal_pointer + 0))
    f === :nbuffer && return unsafe_load(Ptr{Int32}(internal_pointer + 4))
    f === :nplugin && return unsafe_load(Ptr{Int32}(internal_pointer + 8))
    f === :pstack && return unsafe_load(Ptr{UInt64}(internal_pointer + 16))
    f === :parena && return unsafe_load(Ptr{UInt64}(internal_pointer + 24))
    f === :maxuse_stack && return unsafe_load(Ptr{Int32}(internal_pointer + 32))
    f === :maxuse_arena && return unsafe_load(Ptr{UInt64}(internal_pointer + 40))
    f === :maxuse_con && return unsafe_load(Ptr{Int32}(internal_pointer + 48))
    f === :maxuse_efc && return unsafe_load(Ptr{Int32}(internal_pointer + 52))
    f === :warning && return UnsafeArray(Ptr{mjWarningStat_}(internal_pointer + 56), (8,))
    f === :timer && return UnsafeArray(Ptr{mjTimerStat_}(internal_pointer + 120), (13,))
    f === :solver && return UnsafeArray(Ptr{mjSolverStat_}(internal_pointer + 328), (1000,))
    f === :solver_iter && return unsafe_load(Ptr{Int32}(internal_pointer + 40328))
    f === :solver_nnz && return unsafe_load(Ptr{Int32}(internal_pointer + 40332))
    f === :solver_fwdinv && return UnsafeArray(Ptr{Float64}(internal_pointer + 40336), (2,))
    f === :nbodypair_broad && return unsafe_load(Ptr{Int32}(internal_pointer + 40352))
    f === :nbodypair_narrow && return unsafe_load(Ptr{Int32}(internal_pointer + 40356))
    f === :ngeompair_mid && return unsafe_load(Ptr{Int32}(internal_pointer + 40360))
    f === :ngeompair_narrow && return unsafe_load(Ptr{Int32}(internal_pointer + 40364))
    f === :ne && return unsafe_load(Ptr{Int32}(internal_pointer + 40368))
    f === :nf && return unsafe_load(Ptr{Int32}(internal_pointer + 40372))
    f === :nefc && return unsafe_load(Ptr{Int32}(internal_pointer + 40376))
    f === :nnzJ && return unsafe_load(Ptr{Int32}(internal_pointer + 40380))
    f === :ncon && return unsafe_load(Ptr{Int32}(internal_pointer + 40384))
    f === :time && return unsafe_load(Ptr{Float64}(internal_pointer + 40392))
    f === :energy && return UnsafeArray(Ptr{Float64}(internal_pointer + 40400), (2,))
    f === :buffer && return unsafe_load(Ptr{Ptr{Nothing}}(internal_pointer + 40416))
    f === :arena && return unsafe_load(Ptr{Ptr{Nothing}}(internal_pointer + 40424))
    f === :qpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40432)), (Int(1), Int(model.nq))))
    f === :qvel && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40440)), (Int(1), Int(model.nv))))
    f === :act && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40448)), (Int(1), Int(model.na))))
    f === :qacc_warmstart && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40456)), (Int(1), Int(model.nv))))
    f === :plugin_state && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40464)), (Int(1), Int(model.npluginstate))))
    f === :ctrl && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40472)), (Int(1), Int(model.nu))))
    f === :qfrc_applied && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40480)), (Int(1), Int(model.nv))))
    f === :xfrc_applied && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40488)), (Int(6), Int(model.nbody))))
    f === :mocap_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40496)), (Int(3), Int(model.nmocap))))
    f === :mocap_quat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40504)), (Int(4), Int(model.nmocap))))
    f === :qacc && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40512)), (Int(1), Int(model.nv))))
    f === :act_dot && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40520)), (Int(1), Int(model.na))))
    f === :userdata && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40528)), (Int(1), Int(model.nuserdata))))
    f === :sensordata && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40536)), (Int(1), Int(model.nsensordata))))
    f === :plugin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40544)), (Int(1), Int(x.nplugin))))
    f === :plugin_data && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt64}}(internal_pointer + 40552)), (Int(1), Int(x.nplugin))))
    f === :xpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40560)), (Int(3), Int(model.nbody))))
    f === :xquat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40568)), (Int(4), Int(model.nbody))))
    f === :xmat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40576)), (Int(9), Int(model.nbody))))
    f === :xipos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40584)), (Int(3), Int(model.nbody))))
    f === :ximat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40592)), (Int(9), Int(model.nbody))))
    f === :xanchor && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40600)), (Int(3), Int(model.njnt))))
    f === :xaxis && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40608)), (Int(3), Int(model.njnt))))
    f === :geom_xpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40616)), (Int(3), Int(model.ngeom))))
    f === :geom_xmat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40624)), (Int(9), Int(model.ngeom))))
    f === :site_xpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40632)), (Int(3), Int(model.nsite))))
    f === :site_xmat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40640)), (Int(9), Int(model.nsite))))
    f === :cam_xpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40648)), (Int(3), Int(model.ncam))))
    f === :cam_xmat && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40656)), (Int(9), Int(model.ncam))))
    f === :light_xpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40664)), (Int(3), Int(model.nlight))))
    f === :light_xdir && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40672)), (Int(3), Int(model.nlight))))
    f === :subtree_com && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40680)), (Int(3), Int(model.nbody))))
    f === :cdof && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40688)), (Int(6), Int(model.nv))))
    f === :cinert && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40696)), (Int(10), Int(model.nbody))))
    f === :ten_wrapadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40704)), (Int(1), Int(model.ntendon))))
    f === :ten_wrapnum && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40712)), (Int(1), Int(model.ntendon))))
    f === :ten_J_rownnz && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40720)), (Int(1), Int(model.ntendon))))
    f === :ten_J_rowadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40728)), (Int(1), Int(model.ntendon))))
    f === :ten_J_colind && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40736)), (Int(model.nv), Int(model.ntendon))))
    f === :ten_length && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40744)), (Int(1), Int(model.ntendon))))
    f === :ten_J && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40752)), (Int(model.nv), Int(model.ntendon))))
    f === :wrap_obj && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40760)), (Int(2), Int(model.nwrap))))
    f === :wrap_xpos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40768)), (Int(6), Int(model.nwrap))))
    f === :actuator_length && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40776)), (Int(1), Int(model.nu))))
    f === :actuator_moment && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40784)), (Int(model.nv), Int(model.nu))))
    f === :crb && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40792)), (Int(10), Int(model.nbody))))
    f === :qM && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40800)), (Int(1), Int(model.nM))))
    f === :qLD && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40808)), (Int(1), Int(model.nM))))
    f === :qLDiagInv && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40816)), (Int(1), Int(model.nv))))
    f === :qLDiagSqrtInv && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40824)), (Int(1), Int(model.nv))))
    f === :bvh_active && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 40832)), (Int(1), Int(model.nbvh))))
    f === :ten_velocity && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40840)), (Int(1), Int(model.ntendon))))
    f === :actuator_velocity && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40848)), (Int(1), Int(model.nu))))
    f === :cvel && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40856)), (Int(6), Int(model.nbody))))
    f === :cdof_dot && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40864)), (Int(6), Int(model.nv))))
    f === :qfrc_bias && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40872)), (Int(1), Int(model.nv))))
    f === :qfrc_passive && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40880)), (Int(1), Int(model.nv))))
    f === :efc_vel && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40888)), (Int(1), Int(data.nefc))))
    f === :efc_aref && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40896)), (Int(1), Int(data.nefc))))
    f === :subtree_linvel && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40904)), (Int(3), Int(model.nbody))))
    f === :subtree_angmom && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40912)), (Int(3), Int(model.nbody))))
    f === :qH && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40920)), (Int(1), Int(model.nM))))
    f === :qHDiagInv && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40928)), (Int(1), Int(model.nv))))
    f === :D_rownnz && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40936)), (Int(1), Int(model.nv))))
    f === :D_rowadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40944)), (Int(1), Int(model.nv))))
    f === :D_colind && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40952)), (Int(1), Int(model.nD))))
    f === :B_rownnz && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40960)), (Int(1), Int(model.nbody))))
    f === :B_rowadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40968)), (Int(1), Int(model.nbody))))
    f === :B_colind && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40976)), (Int(1), Int(model.nB))))
    f === :qDeriv && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40984)), (Int(1), Int(model.nD))))
    f === :qLU && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40992)), (Int(1), Int(model.nD))))
    f === :actuator_force && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41000)), (Int(1), Int(model.nu))))
    f === :qfrc_actuator && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41008)), (Int(1), Int(model.nv))))
    f === :qfrc_smooth && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41016)), (Int(1), Int(model.nv))))
    f === :qacc_smooth && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41024)), (Int(1), Int(model.nv))))
    f === :qfrc_constraint && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41032)), (Int(1), Int(model.nv))))
    f === :qfrc_inverse && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41040)), (Int(1), Int(model.nv))))
    f === :cacc && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41048)), (Int(6), Int(model.nbody))))
    f === :cfrc_int && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41056)), (Int(6), Int(model.nbody))))
    f === :cfrc_ext && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41064)), (Int(6), Int(model.nbody))))
    f === :contact && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{mjContact_}}(internal_pointer + 41072)), (Int(1), Int(data.ncon))))
    f === :efc_type && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41080)), (Int(1), Int(data.nefc))))
    f === :efc_id && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41088)), (Int(1), Int(data.nefc))))
    f === :efc_J_rownnz && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41096)), (Int(1), Int(data.nefc))))
    f === :efc_J_rowadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41104)), (Int(1), Int(data.nefc))))
    f === :efc_J_rowsuper && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41112)), (Int(1), Int(data.nefc))))
    f === :efc_J_colind && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41120)), (Int(1), Int(data.nnzJ))))
    f === :efc_JT_rownnz && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41128)), (Int(1), Int(model.nv))))
    f === :efc_JT_rowadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41136)), (Int(1), Int(model.nv))))
    f === :efc_JT_rowsuper && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41144)), (Int(1), Int(model.nv))))
    f === :efc_JT_colind && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41152)), (Int(1), Int(data.nnzJ))))
    f === :efc_J && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41160)), (Int(1), Int(data.nnzJ))))
    f === :efc_JT && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41168)), (Int(1), Int(data.nnzJ))))
    f === :efc_pos && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41176)), (Int(1), Int(data.nefc))))
    f === :efc_margin && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41184)), (Int(1), Int(data.nefc))))
    f === :efc_frictionloss && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41192)), (Int(1), Int(data.nefc))))
    f === :efc_diagApprox && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41200)), (Int(1), Int(data.nefc))))
    f === :efc_KBIP && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41208)), (Int(4), Int(data.nefc))))
    f === :efc_D && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41216)), (Int(1), Int(data.nefc))))
    f === :efc_R && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41224)), (Int(1), Int(data.nefc))))
    f === :efc_b && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41232)), (Int(1), Int(data.nefc))))
    f === :efc_force && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41240)), (Int(1), Int(data.nefc))))
    f === :efc_state && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41248)), (Int(1), Int(data.nefc))))
    f === :efc_AR_rownnz && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41256)), (Int(1), Int(data.nefc))))
    f === :efc_AR_rowadr && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41264)), (Int(1), Int(data.nefc))))
    f === :efc_AR_colind && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41272)), (Int(data.nefc), Int(data.nefc))))
    f === :efc_AR && return transpose(UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41280)), (Int(data.nefc), Int(data.nefc))))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::Data, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :nstack
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :nbuffer
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :nplugin
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :pstack
        cvalue = convert(UInt64, value)
        unsafe_store!(Ptr{UInt64}(internal_pointer + 12), cvalue)
        return cvalue
    end
    if f === :parena
        cvalue = convert(UInt64, value)
        unsafe_store!(Ptr{UInt64}(internal_pointer + 20), cvalue)
        return cvalue
    end
    if f === :maxuse_stack
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 28), cvalue)
        return cvalue
    end
    if f === :maxuse_arena
        cvalue = convert(UInt64, value)
        unsafe_store!(Ptr{UInt64}(internal_pointer + 32), cvalue)
        return cvalue
    end
    if f === :maxuse_con
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 40), cvalue)
        return cvalue
    end
    if f === :maxuse_efc
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 44), cvalue)
        return cvalue
    end
    if f === :solver_iter
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 48), cvalue)
        return cvalue
    end
    if f === :solver_nnz
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 52), cvalue)
        return cvalue
    end
    if f === :nbodypair_broad
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 56), cvalue)
        return cvalue
    end
    if f === :nbodypair_narrow
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 60), cvalue)
        return cvalue
    end
    if f === :ngeompair_mid
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 64), cvalue)
        return cvalue
    end
    if f === :ngeompair_narrow
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 68), cvalue)
        return cvalue
    end
    if f === :ne
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 72), cvalue)
        return cvalue
    end
    if f === :nf
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 76), cvalue)
        return cvalue
    end
    if f === :nefc
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 80), cvalue)
        return cvalue
    end
    if f === :nnzJ
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 84), cvalue)
        return cvalue
    end
    if f === :ncon
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 88), cvalue)
        return cvalue
    end
    if f === :time
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 92), cvalue)
        return cvalue
    end
    if f in (:warning, :timer, :solver, :solver_fwdinv, :energy)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    if f in (:buffer, :arena, :qpos, :qvel, :act, :qacc_warmstart, :plugin_state, :ctrl, :qfrc_applied, :xfrc_applied, :mocap_pos, :mocap_quat, :qacc, :act_dot, :userdata, :sensordata, :plugin, :plugin_data, :xpos, :xquat, :xmat, :xipos, :ximat, :xanchor, :xaxis, :geom_xpos, :geom_xmat, :site_xpos, :site_xmat, :cam_xpos, :cam_xmat, :light_xpos, :light_xdir, :subtree_com, :cdof, :cinert, :ten_wrapadr, :ten_wrapnum, :ten_J_rownnz, :ten_J_rowadr, :ten_J_colind, :ten_length, :ten_J, :wrap_obj, :wrap_xpos, :actuator_length, :actuator_moment, :crb, :qM, :qLD, :qLDiagInv, :qLDiagSqrtInv, :bvh_active, :ten_velocity, :actuator_velocity, :cvel, :cdof_dot, :qfrc_bias, :qfrc_passive, :efc_vel, :efc_aref, :subtree_linvel, :subtree_angmom, :qH, :qHDiagInv, :D_rownnz, :D_rowadr, :D_colind, :B_rownnz, :B_rowadr, :B_colind, :qDeriv, :qLU, :actuator_force, :qfrc_actuator, :qfrc_smooth, :qacc_smooth, :qfrc_constraint, :qfrc_inverse, :cacc, :cfrc_int, :cfrc_ext, :contact, :efc_type, :efc_id, :efc_J_rownnz, :efc_J_rowadr, :efc_J_rowsuper, :efc_J_colind, :efc_JT_rownnz, :efc_JT_rowadr, :efc_JT_rowsuper, :efc_JT_colind, :efc_J, :efc_JT, :efc_pos, :efc_margin, :efc_frictionloss, :efc_diagApprox, :efc_KBIP, :efc_D, :efc_R, :efc_b, :efc_force, :efc_state, :efc_AR_rownnz, :efc_AR_rowadr, :efc_AR_colind, :efc_AR)
        error("Cannot overwrite a pointer field.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjData}}, wrapper::Data)
    return wrapper.internal_pointer
end
