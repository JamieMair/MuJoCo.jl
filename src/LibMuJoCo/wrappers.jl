using UnsafeArrays
export Options, Statistics, Model, Data
struct Options
    internal_pointer::Ptr{mjOption}
end
struct Statistics
    internal_pointer::Ptr{mjStatistic}
end
struct Model
    internal_pointer::Ptr{mjModel}
end
struct Data
    internal_pointer::Ptr{mjData}
    model::Model
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
    f === :qpos0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1136)), (Int(x.nq), Int(1)))
    f === :qpos_spring && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1144)), (Int(x.nq), Int(1)))
    f === :body_parentid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1152)), (Int(x.nbody), Int(1)))
    f === :body_rootid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1160)), (Int(x.nbody), Int(1)))
    f === :body_weldid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1168)), (Int(x.nbody), Int(1)))
    f === :body_mocapid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1176)), (Int(x.nbody), Int(1)))
    f === :body_jntnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1184)), (Int(x.nbody), Int(1)))
    f === :body_jntadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1192)), (Int(x.nbody), Int(1)))
    f === :body_dofnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1200)), (Int(x.nbody), Int(1)))
    f === :body_dofadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1208)), (Int(x.nbody), Int(1)))
    f === :body_geomnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1216)), (Int(x.nbody), Int(1)))
    f === :body_geomadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1224)), (Int(x.nbody), Int(1)))
    f === :body_simple && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1232)), (Int(x.nbody), Int(1)))
    f === :body_sameframe && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1240)), (Int(x.nbody), Int(1)))
    f === :body_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1248)), (Int(x.nbody), Int(3)))
    f === :body_quat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1256)), (Int(x.nbody), Int(4)))
    f === :body_ipos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1264)), (Int(x.nbody), Int(3)))
    f === :body_iquat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1272)), (Int(x.nbody), Int(4)))
    f === :body_mass && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1280)), (Int(x.nbody), Int(1)))
    f === :body_subtreemass && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1288)), (Int(x.nbody), Int(1)))
    f === :body_inertia && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1296)), (Int(x.nbody), Int(3)))
    f === :body_invweight0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1304)), (Int(x.nbody), Int(2)))
    f === :body_gravcomp && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1312)), (Int(x.nbody), Int(1)))
    f === :body_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1320)), (Int(x.nbody), Int(model.nuser_body)))
    f === :body_plugin && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1328)), (Int(x.nbody), Int(1)))
    f === :body_bvhadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1336)), (Int(x.nbody), Int(1)))
    f === :body_bvhnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1344)), (Int(x.nbody), Int(1)))
    f === :bvh_depth && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1352)), (Int(x.nbvh), Int(1)))
    f === :bvh_child && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1360)), (Int(x.nbvh), Int(2)))
    f === :bvh_geomid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1368)), (Int(x.nbvh), Int(1)))
    f === :bvh_aabb && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1376)), (Int(x.nbvh), Int(6)))
    f === :jnt_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1384)), (Int(x.njnt), Int(1)))
    f === :jnt_qposadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1392)), (Int(x.njnt), Int(1)))
    f === :jnt_dofadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1400)), (Int(x.njnt), Int(1)))
    f === :jnt_bodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1408)), (Int(x.njnt), Int(1)))
    f === :jnt_group && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1416)), (Int(x.njnt), Int(1)))
    f === :jnt_limited && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1424)), (Int(x.njnt), Int(1)))
    f === :jnt_actfrclimited && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1432)), (Int(x.njnt), Int(1)))
    f === :jnt_solref && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1440)), (Int(x.njnt), Int(x.mjNREF)))
    f === :jnt_solimp && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1448)), (Int(x.njnt), Int(x.mjNIMP)))
    f === :jnt_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1456)), (Int(x.njnt), Int(3)))
    f === :jnt_axis && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1464)), (Int(x.njnt), Int(3)))
    f === :jnt_stiffness && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1472)), (Int(x.njnt), Int(1)))
    f === :jnt_range && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1480)), (Int(x.njnt), Int(2)))
    f === :jnt_actfrcrange && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1488)), (Int(x.njnt), Int(2)))
    f === :jnt_margin && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1496)), (Int(x.njnt), Int(1)))
    f === :jnt_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1504)), (Int(x.njnt), Int(model.nuser_jnt)))
    f === :dof_bodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1512)), (Int(x.nv), Int(1)))
    f === :dof_jntid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1520)), (Int(x.nv), Int(1)))
    f === :dof_parentid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1528)), (Int(x.nv), Int(1)))
    f === :dof_Madr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1536)), (Int(x.nv), Int(1)))
    f === :dof_simplenum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1544)), (Int(x.nv), Int(1)))
    f === :dof_solref && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1552)), (Int(x.nv), Int(x.mjNREF)))
    f === :dof_solimp && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1560)), (Int(x.nv), Int(x.mjNIMP)))
    f === :dof_frictionloss && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1568)), (Int(x.nv), Int(1)))
    f === :dof_armature && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1576)), (Int(x.nv), Int(1)))
    f === :dof_damping && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1584)), (Int(x.nv), Int(1)))
    f === :dof_invweight0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1592)), (Int(x.nv), Int(1)))
    f === :dof_M0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1600)), (Int(x.nv), Int(1)))
    f === :geom_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1608)), (Int(x.ngeom), Int(1)))
    f === :geom_contype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1616)), (Int(x.ngeom), Int(1)))
    f === :geom_conaffinity && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1624)), (Int(x.ngeom), Int(1)))
    f === :geom_condim && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1632)), (Int(x.ngeom), Int(1)))
    f === :geom_bodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1640)), (Int(x.ngeom), Int(1)))
    f === :geom_dataid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1648)), (Int(x.ngeom), Int(1)))
    f === :geom_matid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1656)), (Int(x.ngeom), Int(1)))
    f === :geom_group && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1664)), (Int(x.ngeom), Int(1)))
    f === :geom_priority && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1672)), (Int(x.ngeom), Int(1)))
    f === :geom_sameframe && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1680)), (Int(x.ngeom), Int(1)))
    f === :geom_solmix && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1688)), (Int(x.ngeom), Int(1)))
    f === :geom_solref && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1696)), (Int(x.ngeom), Int(x.mjNREF)))
    f === :geom_solimp && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1704)), (Int(x.ngeom), Int(x.mjNIMP)))
    f === :geom_size && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1712)), (Int(x.ngeom), Int(3)))
    f === :geom_aabb && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1720)), (Int(x.ngeom), Int(6)))
    f === :geom_rbound && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1728)), (Int(x.ngeom), Int(1)))
    f === :geom_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1736)), (Int(x.ngeom), Int(3)))
    f === :geom_quat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1744)), (Int(x.ngeom), Int(4)))
    f === :geom_friction && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1752)), (Int(x.ngeom), Int(3)))
    f === :geom_margin && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1760)), (Int(x.ngeom), Int(1)))
    f === :geom_gap && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1768)), (Int(x.ngeom), Int(1)))
    f === :geom_fluid && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1776)), (Int(x.ngeom), Int(x.mjNFLUID)))
    f === :geom_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1784)), (Int(x.ngeom), Int(model.nuser_geom)))
    f === :geom_rgba && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 1792)), (Int(x.ngeom), Int(4)))
    f === :site_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1800)), (Int(x.nsite), Int(1)))
    f === :site_bodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1808)), (Int(x.nsite), Int(1)))
    f === :site_matid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1816)), (Int(x.nsite), Int(1)))
    f === :site_group && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1824)), (Int(x.nsite), Int(1)))
    f === :site_sameframe && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1832)), (Int(x.nsite), Int(1)))
    f === :site_size && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1840)), (Int(x.nsite), Int(3)))
    f === :site_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1848)), (Int(x.nsite), Int(3)))
    f === :site_quat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1856)), (Int(x.nsite), Int(4)))
    f === :site_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1864)), (Int(x.nsite), Int(model.nuser_site)))
    f === :site_rgba && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 1872)), (Int(x.nsite), Int(4)))
    f === :cam_mode && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1880)), (Int(x.ncam), Int(1)))
    f === :cam_bodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1888)), (Int(x.ncam), Int(1)))
    f === :cam_targetbodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1896)), (Int(x.ncam), Int(1)))
    f === :cam_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1904)), (Int(x.ncam), Int(3)))
    f === :cam_quat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1912)), (Int(x.ncam), Int(4)))
    f === :cam_poscom0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1920)), (Int(x.ncam), Int(3)))
    f === :cam_pos0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1928)), (Int(x.ncam), Int(3)))
    f === :cam_mat0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1936)), (Int(x.ncam), Int(9)))
    f === :cam_fovy && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1944)), (Int(x.ncam), Int(1)))
    f === :cam_ipd && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1952)), (Int(x.ncam), Int(1)))
    f === :cam_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1960)), (Int(x.ncam), Int(model.nuser_cam)))
    f === :light_mode && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1968)), (Int(x.nlight), Int(1)))
    f === :light_bodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1976)), (Int(x.nlight), Int(1)))
    f === :light_targetbodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1984)), (Int(x.nlight), Int(1)))
    f === :light_directional && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1992)), (Int(x.nlight), Int(1)))
    f === :light_castshadow && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2000)), (Int(x.nlight), Int(1)))
    f === :light_active && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2008)), (Int(x.nlight), Int(1)))
    f === :light_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2016)), (Int(x.nlight), Int(3)))
    f === :light_dir && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2024)), (Int(x.nlight), Int(3)))
    f === :light_poscom0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2032)), (Int(x.nlight), Int(3)))
    f === :light_pos0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2040)), (Int(x.nlight), Int(3)))
    f === :light_dir0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2048)), (Int(x.nlight), Int(3)))
    f === :light_attenuation && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2056)), (Int(x.nlight), Int(3)))
    f === :light_cutoff && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2064)), (Int(x.nlight), Int(1)))
    f === :light_exponent && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2072)), (Int(x.nlight), Int(1)))
    f === :light_ambient && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2080)), (Int(x.nlight), Int(3)))
    f === :light_diffuse && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2088)), (Int(x.nlight), Int(3)))
    f === :light_specular && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2096)), (Int(x.nlight), Int(3)))
    f === :mesh_vertadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2104)), (Int(x.nmesh), Int(1)))
    f === :mesh_vertnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2112)), (Int(x.nmesh), Int(1)))
    f === :mesh_faceadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2120)), (Int(x.nmesh), Int(1)))
    f === :mesh_facenum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2128)), (Int(x.nmesh), Int(1)))
    f === :mesh_bvhadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2136)), (Int(x.nmesh), Int(1)))
    f === :mesh_bvhnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2144)), (Int(x.nmesh), Int(1)))
    f === :mesh_normaladr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2152)), (Int(x.nmesh), Int(1)))
    f === :mesh_normalnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2160)), (Int(x.nmesh), Int(1)))
    f === :mesh_texcoordadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2168)), (Int(x.nmesh), Int(1)))
    f === :mesh_texcoordnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2176)), (Int(x.nmesh), Int(1)))
    f === :mesh_graphadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2184)), (Int(x.nmesh), Int(1)))
    f === :mesh_vert && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2192)), (Int(x.nmeshvert), Int(3)))
    f === :mesh_normal && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2200)), (Int(x.nmeshnormal), Int(3)))
    f === :mesh_texcoord && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2208)), (Int(x.nmeshtexcoord), Int(2)))
    f === :mesh_face && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2216)), (Int(x.nmeshface), Int(3)))
    f === :mesh_facenormal && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2224)), (Int(x.nmeshface), Int(3)))
    f === :mesh_facetexcoord && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2232)), (Int(x.nmeshface), Int(3)))
    f === :mesh_graph && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2240)), (Int(x.nmeshgraph), Int(1)))
    f === :skin_matid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2248)), (Int(x.nskin), Int(1)))
    f === :skin_group && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2256)), (Int(x.nskin), Int(1)))
    f === :skin_rgba && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2264)), (Int(x.nskin), Int(4)))
    f === :skin_inflate && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2272)), (Int(x.nskin), Int(1)))
    f === :skin_vertadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2280)), (Int(x.nskin), Int(1)))
    f === :skin_vertnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2288)), (Int(x.nskin), Int(1)))
    f === :skin_texcoordadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2296)), (Int(x.nskin), Int(1)))
    f === :skin_faceadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2304)), (Int(x.nskin), Int(1)))
    f === :skin_facenum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2312)), (Int(x.nskin), Int(1)))
    f === :skin_boneadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2320)), (Int(x.nskin), Int(1)))
    f === :skin_bonenum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2328)), (Int(x.nskin), Int(1)))
    f === :skin_vert && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2336)), (Int(x.nskinvert), Int(3)))
    f === :skin_texcoord && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2344)), (Int(x.nskintexvert), Int(2)))
    f === :skin_face && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2352)), (Int(x.nskinface), Int(3)))
    f === :skin_bonevertadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2360)), (Int(x.nskinbone), Int(1)))
    f === :skin_bonevertnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2368)), (Int(x.nskinbone), Int(1)))
    f === :skin_bonebindpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2376)), (Int(x.nskinbone), Int(3)))
    f === :skin_bonebindquat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2384)), (Int(x.nskinbone), Int(4)))
    f === :skin_bonebodyid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2392)), (Int(x.nskinbone), Int(1)))
    f === :skin_bonevertid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2400)), (Int(x.nskinbonevert), Int(1)))
    f === :skin_bonevertweight && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2408)), (Int(x.nskinbonevert), Int(1)))
    f === :hfield_size && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2416)), (Int(x.nhfield), Int(4)))
    f === :hfield_nrow && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2424)), (Int(x.nhfield), Int(1)))
    f === :hfield_ncol && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2432)), (Int(x.nhfield), Int(1)))
    f === :hfield_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2440)), (Int(x.nhfield), Int(1)))
    f === :hfield_data && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2448)), (Int(x.nhfielddata), Int(1)))
    f === :tex_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2456)), (Int(x.ntex), Int(1)))
    f === :tex_height && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2464)), (Int(x.ntex), Int(1)))
    f === :tex_width && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2472)), (Int(x.ntex), Int(1)))
    f === :tex_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2480)), (Int(x.ntex), Int(1)))
    f === :tex_rgb && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2488)), (Int(x.ntexdata), Int(1)))
    f === :mat_texid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2496)), (Int(x.nmat), Int(1)))
    f === :mat_texuniform && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2504)), (Int(x.nmat), Int(1)))
    f === :mat_texrepeat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2512)), (Int(x.nmat), Int(2)))
    f === :mat_emission && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2520)), (Int(x.nmat), Int(1)))
    f === :mat_specular && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2528)), (Int(x.nmat), Int(1)))
    f === :mat_shininess && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2536)), (Int(x.nmat), Int(1)))
    f === :mat_reflectance && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2544)), (Int(x.nmat), Int(1)))
    f === :mat_rgba && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2552)), (Int(x.nmat), Int(4)))
    f === :pair_dim && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2560)), (Int(x.npair), Int(1)))
    f === :pair_geom1 && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2568)), (Int(x.npair), Int(1)))
    f === :pair_geom2 && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2576)), (Int(x.npair), Int(1)))
    f === :pair_signature && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2584)), (Int(x.npair), Int(1)))
    f === :pair_solref && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2592)), (Int(x.npair), Int(x.mjNREF)))
    f === :pair_solreffriction && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2600)), (Int(x.npair), Int(x.mjNREF)))
    f === :pair_solimp && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2608)), (Int(x.npair), Int(x.mjNIMP)))
    f === :pair_margin && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2616)), (Int(x.npair), Int(1)))
    f === :pair_gap && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2624)), (Int(x.npair), Int(1)))
    f === :pair_friction && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2632)), (Int(x.npair), Int(5)))
    f === :exclude_signature && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2640)), (Int(x.nexclude), Int(1)))
    f === :eq_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2648)), (Int(x.neq), Int(1)))
    f === :eq_obj1id && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2656)), (Int(x.neq), Int(1)))
    f === :eq_obj2id && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2664)), (Int(x.neq), Int(1)))
    f === :eq_active && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2672)), (Int(x.neq), Int(1)))
    f === :eq_solref && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2680)), (Int(x.neq), Int(x.mjNREF)))
    f === :eq_solimp && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2688)), (Int(x.neq), Int(x.mjNIMP)))
    f === :eq_data && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2696)), (Int(x.neq), Int(x.mjNEQDATA)))
    f === :tendon_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2704)), (Int(x.ntendon), Int(1)))
    f === :tendon_num && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2712)), (Int(x.ntendon), Int(1)))
    f === :tendon_matid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2720)), (Int(x.ntendon), Int(1)))
    f === :tendon_group && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2728)), (Int(x.ntendon), Int(1)))
    f === :tendon_limited && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2736)), (Int(x.ntendon), Int(1)))
    f === :tendon_width && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2744)), (Int(x.ntendon), Int(1)))
    f === :tendon_solref_lim && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2752)), (Int(x.ntendon), Int(x.mjNREF)))
    f === :tendon_solimp_lim && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2760)), (Int(x.ntendon), Int(x.mjNIMP)))
    f === :tendon_solref_fri && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2768)), (Int(x.ntendon), Int(x.mjNREF)))
    f === :tendon_solimp_fri && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2776)), (Int(x.ntendon), Int(x.mjNIMP)))
    f === :tendon_range && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2784)), (Int(x.ntendon), Int(2)))
    f === :tendon_margin && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2792)), (Int(x.ntendon), Int(1)))
    f === :tendon_stiffness && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2800)), (Int(x.ntendon), Int(1)))
    f === :tendon_damping && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2808)), (Int(x.ntendon), Int(1)))
    f === :tendon_frictionloss && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2816)), (Int(x.ntendon), Int(1)))
    f === :tendon_lengthspring && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2824)), (Int(x.ntendon), Int(2)))
    f === :tendon_length0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2832)), (Int(x.ntendon), Int(1)))
    f === :tendon_invweight0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2840)), (Int(x.ntendon), Int(1)))
    f === :tendon_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2848)), (Int(x.ntendon), Int(model.nuser_tendon)))
    f === :tendon_rgba && return UnsafeArray(unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2856)), (Int(x.ntendon), Int(4)))
    f === :wrap_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2864)), (Int(x.nwrap), Int(1)))
    f === :wrap_objid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2872)), (Int(x.nwrap), Int(1)))
    f === :wrap_prm && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2880)), (Int(x.nwrap), Int(1)))
    f === :actuator_trntype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2888)), (Int(x.nu), Int(1)))
    f === :actuator_dyntype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2896)), (Int(x.nu), Int(1)))
    f === :actuator_gaintype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2904)), (Int(x.nu), Int(1)))
    f === :actuator_biastype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2912)), (Int(x.nu), Int(1)))
    f === :actuator_trnid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2920)), (Int(x.nu), Int(2)))
    f === :actuator_actadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2928)), (Int(x.nu), Int(1)))
    f === :actuator_actnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2936)), (Int(x.nu), Int(1)))
    f === :actuator_group && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2944)), (Int(x.nu), Int(1)))
    f === :actuator_ctrllimited && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2952)), (Int(x.nu), Int(1)))
    f === :actuator_forcelimited && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2960)), (Int(x.nu), Int(1)))
    f === :actuator_actlimited && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2968)), (Int(x.nu), Int(1)))
    f === :actuator_dynprm && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2976)), (Int(x.nu), Int(x.mjNDYN)))
    f === :actuator_gainprm && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2984)), (Int(x.nu), Int(x.mjNGAIN)))
    f === :actuator_biasprm && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2992)), (Int(x.nu), Int(x.mjNBIAS)))
    f === :actuator_ctrlrange && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3000)), (Int(x.nu), Int(2)))
    f === :actuator_forcerange && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3008)), (Int(x.nu), Int(2)))
    f === :actuator_actrange && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3016)), (Int(x.nu), Int(2)))
    f === :actuator_gear && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3024)), (Int(x.nu), Int(6)))
    f === :actuator_cranklength && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3032)), (Int(x.nu), Int(1)))
    f === :actuator_acc0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3040)), (Int(x.nu), Int(1)))
    f === :actuator_length0 && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3048)), (Int(x.nu), Int(1)))
    f === :actuator_lengthrange && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3056)), (Int(x.nu), Int(2)))
    f === :actuator_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3064)), (Int(x.nu), Int(model.nuser_actuator)))
    f === :actuator_plugin && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3072)), (Int(x.nu), Int(1)))
    f === :sensor_type && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3080)), (Int(x.nsensor), Int(1)))
    f === :sensor_datatype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3088)), (Int(x.nsensor), Int(1)))
    f === :sensor_needstage && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3096)), (Int(x.nsensor), Int(1)))
    f === :sensor_objtype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3104)), (Int(x.nsensor), Int(1)))
    f === :sensor_objid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3112)), (Int(x.nsensor), Int(1)))
    f === :sensor_reftype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3120)), (Int(x.nsensor), Int(1)))
    f === :sensor_refid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3128)), (Int(x.nsensor), Int(1)))
    f === :sensor_dim && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3136)), (Int(x.nsensor), Int(1)))
    f === :sensor_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3144)), (Int(x.nsensor), Int(1)))
    f === :sensor_cutoff && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3152)), (Int(x.nsensor), Int(1)))
    f === :sensor_noise && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3160)), (Int(x.nsensor), Int(1)))
    f === :sensor_user && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3168)), (Int(x.nsensor), Int(model.nuser_sensor)))
    f === :sensor_plugin && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3176)), (Int(x.nsensor), Int(1)))
    f === :plugin && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3184)), (Int(x.nplugin), Int(1)))
    f === :plugin_stateadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3192)), (Int(x.nplugin), Int(1)))
    f === :plugin_statenum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3200)), (Int(x.nplugin), Int(1)))
    f === :plugin_attr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3208)), (Int(x.npluginattr), Int(1)))
    f === :plugin_attradr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3216)), (Int(x.nplugin), Int(1)))
    f === :numeric_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3224)), (Int(x.nnumeric), Int(1)))
    f === :numeric_size && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3232)), (Int(x.nnumeric), Int(1)))
    f === :numeric_data && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3240)), (Int(x.nnumericdata), Int(1)))
    f === :text_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3248)), (Int(x.ntext), Int(1)))
    f === :text_size && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3256)), (Int(x.ntext), Int(1)))
    f === :text_data && return UnsafeArray(unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3264)), (Int(x.ntextdata), Int(1)))
    f === :tuple_adr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3272)), (Int(x.ntuple), Int(1)))
    f === :tuple_size && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3280)), (Int(x.ntuple), Int(1)))
    f === :tuple_objtype && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3288)), (Int(x.ntupledata), Int(1)))
    f === :tuple_objid && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3296)), (Int(x.ntupledata), Int(1)))
    f === :tuple_objprm && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3304)), (Int(x.ntupledata), Int(1)))
    f === :key_time && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3312)), (Int(x.nkey), Int(1)))
    f === :key_qpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3320)), (Int(x.nkey), Int(model.nq)))
    f === :key_qvel && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3328)), (Int(x.nkey), Int(model.nv)))
    f === :key_act && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3336)), (Int(x.nkey), Int(model.na)))
    f === :key_mpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3344)), (Int(x.nkey), Int(model.nmocap * 3)))
    f === :key_mquat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3352)), (Int(x.nkey), Int(model.nmocap * 4)))
    f === :key_ctrl && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3360)), (Int(x.nkey), Int(model.nu)))
    f === :name_bodyadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3368)), (Int(x.nbody), Int(1)))
    f === :name_jntadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3376)), (Int(x.njnt), Int(1)))
    f === :name_geomadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3384)), (Int(x.ngeom), Int(1)))
    f === :name_siteadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3392)), (Int(x.nsite), Int(1)))
    f === :name_camadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3400)), (Int(x.ncam), Int(1)))
    f === :name_lightadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3408)), (Int(x.nlight), Int(1)))
    f === :name_meshadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3416)), (Int(x.nmesh), Int(1)))
    f === :name_skinadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3424)), (Int(x.nskin), Int(1)))
    f === :name_hfieldadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3432)), (Int(x.nhfield), Int(1)))
    f === :name_texadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3440)), (Int(x.ntex), Int(1)))
    f === :name_matadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3448)), (Int(x.nmat), Int(1)))
    f === :name_pairadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3456)), (Int(x.npair), Int(1)))
    f === :name_excludeadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3464)), (Int(x.nexclude), Int(1)))
    f === :name_eqadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3472)), (Int(x.neq), Int(1)))
    f === :name_tendonadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3480)), (Int(x.ntendon), Int(1)))
    f === :name_actuatoradr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3488)), (Int(x.nu), Int(1)))
    f === :name_sensoradr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3496)), (Int(x.nsensor), Int(1)))
    f === :name_numericadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3504)), (Int(x.nnumeric), Int(1)))
    f === :name_textadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3512)), (Int(x.ntext), Int(1)))
    f === :name_tupleadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3520)), (Int(x.ntuple), Int(1)))
    f === :name_keyadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3528)), (Int(x.nkey), Int(1)))
    f === :name_pluginadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3536)), (Int(x.nplugin), Int(1)))
    f === :names && return UnsafeArray(unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3544)), (Int(x.nnames), Int(1)))
    f === :names_map && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3552)), (Int(x.nnames_map), Int(1)))
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
function Base.propertynames(x::Data)
    (:nstack, :nbuffer, :nplugin, :pstack, :parena, :maxuse_stack, :maxuse_arena, :maxuse_con, :maxuse_efc, :warning, :timer, :solver, :solver_iter, :solver_nnz, :solver_fwdinv, :nbodypair_broad, :nbodypair_narrow, :ngeompair_mid, :ngeompair_narrow, :ne, :nf, :nefc, :nnzJ, :ncon, :time, :energy, :buffer, :arena, :qpos, :qvel, :act, :qacc_warmstart, :plugin_state, :ctrl, :qfrc_applied, :xfrc_applied, :mocap_pos, :mocap_quat, :qacc, :act_dot, :userdata, :sensordata, :plugin, :plugin_data, :xpos, :xquat, :xmat, :xipos, :ximat, :xanchor, :xaxis, :geom_xpos, :geom_xmat, :site_xpos, :site_xmat, :cam_xpos, :cam_xmat, :light_xpos, :light_xdir, :subtree_com, :cdof, :cinert, :ten_wrapadr, :ten_wrapnum, :ten_J_rownnz, :ten_J_rowadr, :ten_J_colind, :ten_length, :ten_J, :wrap_obj, :wrap_xpos, :actuator_length, :actuator_moment, :crb, :qM, :qLD, :qLDiagInv, :qLDiagSqrtInv, :bvh_active, :ten_velocity, :actuator_velocity, :cvel, :cdof_dot, :qfrc_bias, :qfrc_passive, :efc_vel, :efc_aref, :subtree_linvel, :subtree_angmom, :qH, :qHDiagInv, :D_rownnz, :D_rowadr, :D_colind, :B_rownnz, :B_rowadr, :B_colind, :qDeriv, :qLU, :actuator_force, :qfrc_actuator, :qfrc_smooth, :qacc_smooth, :qfrc_constraint, :qfrc_inverse, :cacc, :cfrc_int, :cfrc_ext, :contact, :efc_type, :efc_id, :efc_J_rownnz, :efc_J_rowadr, :efc_J_rowsuper, :efc_J_colind, :efc_JT_rownnz, :efc_JT_rowadr, :efc_JT_rowsuper, :efc_JT_colind, :efc_J, :efc_JT, :efc_pos, :efc_margin, :efc_frictionloss, :efc_diagApprox, :efc_KBIP, :efc_D, :efc_R, :efc_b, :efc_force, :efc_state, :efc_AR_rownnz, :efc_AR_rowadr, :efc_AR_colind, :efc_AR)
end
function Base.getproperty(x::Data, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    model = getfield(x, :model)
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
    f === :qpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40432)), (Int(x.nq), Int(1)))
    f === :qvel && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40440)), (Int(x.nv), Int(1)))
    f === :act && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40448)), (Int(x.na), Int(1)))
    f === :qacc_warmstart && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40456)), (Int(x.nv), Int(1)))
    f === :plugin_state && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40464)), (Int(x.npluginstate), Int(1)))
    f === :ctrl && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40472)), (Int(x.nu), Int(1)))
    f === :qfrc_applied && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40480)), (Int(x.nv), Int(1)))
    f === :xfrc_applied && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40488)), (Int(x.nbody), Int(6)))
    f === :mocap_pos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40496)), (Int(x.nmocap), Int(3)))
    f === :mocap_quat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40504)), (Int(x.nmocap), Int(4)))
    f === :qacc && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40512)), (Int(x.nv), Int(1)))
    f === :act_dot && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40520)), (Int(x.na), Int(1)))
    f === :userdata && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40528)), (Int(x.nuserdata), Int(1)))
    f === :sensordata && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40536)), (Int(x.nsensordata), Int(1)))
    f === :plugin && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40544)), (Int(x.nplugin), Int(1)))
    f === :plugin_data && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt64}}(internal_pointer + 40552)), (Int(x.nplugin), Int(1)))
    f === :xpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40560)), (Int(x.nbody), Int(3)))
    f === :xquat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40568)), (Int(x.nbody), Int(4)))
    f === :xmat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40576)), (Int(x.nbody), Int(9)))
    f === :xipos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40584)), (Int(x.nbody), Int(3)))
    f === :ximat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40592)), (Int(x.nbody), Int(9)))
    f === :xanchor && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40600)), (Int(x.njnt), Int(3)))
    f === :xaxis && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40608)), (Int(x.njnt), Int(3)))
    f === :geom_xpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40616)), (Int(x.ngeom), Int(3)))
    f === :geom_xmat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40624)), (Int(x.ngeom), Int(9)))
    f === :site_xpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40632)), (Int(x.nsite), Int(3)))
    f === :site_xmat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40640)), (Int(x.nsite), Int(9)))
    f === :cam_xpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40648)), (Int(x.ncam), Int(3)))
    f === :cam_xmat && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40656)), (Int(x.ncam), Int(9)))
    f === :light_xpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40664)), (Int(x.nlight), Int(3)))
    f === :light_xdir && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40672)), (Int(x.nlight), Int(3)))
    f === :subtree_com && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40680)), (Int(x.nbody), Int(3)))
    f === :cdof && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40688)), (Int(x.nv), Int(6)))
    f === :cinert && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40696)), (Int(x.nbody), Int(10)))
    f === :ten_wrapadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40704)), (Int(x.ntendon), Int(1)))
    f === :ten_wrapnum && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40712)), (Int(x.ntendon), Int(1)))
    f === :ten_J_rownnz && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40720)), (Int(x.ntendon), Int(1)))
    f === :ten_J_rowadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40728)), (Int(x.ntendon), Int(1)))
    f === :ten_J_colind && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40736)), (Int(x.ntendon), Int(model.nv)))
    f === :ten_length && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40744)), (Int(x.ntendon), Int(1)))
    f === :ten_J && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40752)), (Int(x.ntendon), Int(model.nv)))
    f === :wrap_obj && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40760)), (Int(x.nwrap), Int(2)))
    f === :wrap_xpos && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40768)), (Int(x.nwrap), Int(6)))
    f === :actuator_length && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40776)), (Int(x.nu), Int(1)))
    f === :actuator_moment && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40784)), (Int(x.nu), Int(model.nv)))
    f === :crb && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40792)), (Int(x.nbody), Int(10)))
    f === :qM && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40800)), (Int(x.nM), Int(1)))
    f === :qLD && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40808)), (Int(x.nM), Int(1)))
    f === :qLDiagInv && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40816)), (Int(x.nv), Int(1)))
    f === :qLDiagSqrtInv && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40824)), (Int(x.nv), Int(1)))
    f === :bvh_active && return UnsafeArray(unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 40832)), (Int(x.nbvh), Int(1)))
    f === :ten_velocity && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40840)), (Int(x.ntendon), Int(1)))
    f === :actuator_velocity && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40848)), (Int(x.nu), Int(1)))
    f === :cvel && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40856)), (Int(x.nbody), Int(6)))
    f === :cdof_dot && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40864)), (Int(x.nv), Int(6)))
    f === :qfrc_bias && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40872)), (Int(x.nv), Int(1)))
    f === :qfrc_passive && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40880)), (Int(x.nv), Int(1)))
    f === :efc_vel && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40888))
    f === :efc_aref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40896))
    f === :subtree_linvel && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40904)), (Int(x.nbody), Int(3)))
    f === :subtree_angmom && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40912)), (Int(x.nbody), Int(3)))
    f === :qH && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40920)), (Int(x.nM), Int(1)))
    f === :qHDiagInv && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40928)), (Int(x.nv), Int(1)))
    f === :D_rownnz && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40936)), (Int(x.nv), Int(1)))
    f === :D_rowadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40944)), (Int(x.nv), Int(1)))
    f === :D_colind && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40952)), (Int(x.nD), Int(1)))
    f === :B_rownnz && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40960)), (Int(x.nbody), Int(1)))
    f === :B_rowadr && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40968)), (Int(x.nbody), Int(1)))
    f === :B_colind && return UnsafeArray(unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40976)), (Int(x.nB), Int(1)))
    f === :qDeriv && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40984)), (Int(x.nD), Int(1)))
    f === :qLU && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40992)), (Int(x.nD), Int(1)))
    f === :actuator_force && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41000)), (Int(x.nu), Int(1)))
    f === :qfrc_actuator && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41008)), (Int(x.nv), Int(1)))
    f === :qfrc_smooth && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41016)), (Int(x.nv), Int(1)))
    f === :qacc_smooth && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41024)), (Int(x.nv), Int(1)))
    f === :qfrc_constraint && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41032)), (Int(x.nv), Int(1)))
    f === :qfrc_inverse && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41040)), (Int(x.nv), Int(1)))
    f === :cacc && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41048)), (Int(x.nbody), Int(6)))
    f === :cfrc_int && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41056)), (Int(x.nbody), Int(6)))
    f === :cfrc_ext && return UnsafeArray(unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41064)), (Int(x.nbody), Int(6)))
    f === :contact && return unsafe_load(Ptr{Ptr{mjContact_}}(internal_pointer + 41072))
    f === :efc_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41080))
    f === :efc_id && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41088))
    f === :efc_J_rownnz && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41096))
    f === :efc_J_rowadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41104))
    f === :efc_J_rowsuper && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41112))
    f === :efc_J_colind && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41120))
    f === :efc_JT_rownnz && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41128))
    f === :efc_JT_rowadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41136))
    f === :efc_JT_rowsuper && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41144))
    f === :efc_JT_colind && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41152))
    f === :efc_J && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41160))
    f === :efc_JT && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41168))
    f === :efc_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41176))
    f === :efc_margin && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41184))
    f === :efc_frictionloss && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41192))
    f === :efc_diagApprox && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41200))
    f === :efc_KBIP && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41208))
    f === :efc_D && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41216))
    f === :efc_R && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41224))
    f === :efc_b && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41232))
    f === :efc_force && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41240))
    f === :efc_state && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41248))
    f === :efc_AR_rownnz && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41256))
    f === :efc_AR_rowadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41264))
    f === :efc_AR_colind && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 41272))
    f === :efc_AR && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41280))
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
