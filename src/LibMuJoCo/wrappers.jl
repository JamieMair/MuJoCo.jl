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
    f === :qpos0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1136))
    f === :qpos_spring && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1144))
    f === :body_parentid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1152))
    f === :body_rootid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1160))
    f === :body_weldid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1168))
    f === :body_mocapid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1176))
    f === :body_jntnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1184))
    f === :body_jntadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1192))
    f === :body_dofnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1200))
    f === :body_dofadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1208))
    f === :body_geomnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1216))
    f === :body_geomadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1224))
    f === :body_simple && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1232))
    f === :body_sameframe && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1240))
    f === :body_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1248))
    f === :body_quat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1256))
    f === :body_ipos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1264))
    f === :body_iquat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1272))
    f === :body_mass && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1280))
    f === :body_subtreemass && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1288))
    f === :body_inertia && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1296))
    f === :body_invweight0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1304))
    f === :body_gravcomp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1312))
    f === :body_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1320))
    f === :body_plugin && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1328))
    f === :body_bvhadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1336))
    f === :body_bvhnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1344))
    f === :bvh_depth && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1352))
    f === :bvh_child && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1360))
    f === :bvh_geomid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1368))
    f === :bvh_aabb && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1376))
    f === :jnt_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1384))
    f === :jnt_qposadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1392))
    f === :jnt_dofadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1400))
    f === :jnt_bodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1408))
    f === :jnt_group && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1416))
    f === :jnt_limited && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1424))
    f === :jnt_actfrclimited && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1432))
    f === :jnt_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1440))
    f === :jnt_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1448))
    f === :jnt_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1456))
    f === :jnt_axis && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1464))
    f === :jnt_stiffness && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1472))
    f === :jnt_range && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1480))
    f === :jnt_actfrcrange && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1488))
    f === :jnt_margin && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1496))
    f === :jnt_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1504))
    f === :dof_bodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1512))
    f === :dof_jntid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1520))
    f === :dof_parentid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1528))
    f === :dof_Madr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1536))
    f === :dof_simplenum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1544))
    f === :dof_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1552))
    f === :dof_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1560))
    f === :dof_frictionloss && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1568))
    f === :dof_armature && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1576))
    f === :dof_damping && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1584))
    f === :dof_invweight0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1592))
    f === :dof_M0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1600))
    f === :geom_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1608))
    f === :geom_contype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1616))
    f === :geom_conaffinity && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1624))
    f === :geom_condim && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1632))
    f === :geom_bodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1640))
    f === :geom_dataid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1648))
    f === :geom_matid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1656))
    f === :geom_group && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1664))
    f === :geom_priority && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1672))
    f === :geom_sameframe && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1680))
    f === :geom_solmix && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1688))
    f === :geom_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1696))
    f === :geom_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1704))
    f === :geom_size && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1712))
    f === :geom_aabb && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1720))
    f === :geom_rbound && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1728))
    f === :geom_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1736))
    f === :geom_quat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1744))
    f === :geom_friction && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1752))
    f === :geom_margin && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1760))
    f === :geom_gap && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1768))
    f === :geom_fluid && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1776))
    f === :geom_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1784))
    f === :geom_rgba && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 1792))
    f === :site_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1800))
    f === :site_bodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1808))
    f === :site_matid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1816))
    f === :site_group && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1824))
    f === :site_sameframe && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1832))
    f === :site_size && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1840))
    f === :site_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1848))
    f === :site_quat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1856))
    f === :site_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1864))
    f === :site_rgba && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 1872))
    f === :cam_mode && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1880))
    f === :cam_bodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1888))
    f === :cam_targetbodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1896))
    f === :cam_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1904))
    f === :cam_quat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1912))
    f === :cam_poscom0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1920))
    f === :cam_pos0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1928))
    f === :cam_mat0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1936))
    f === :cam_fovy && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1944))
    f === :cam_ipd && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1952))
    f === :cam_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 1960))
    f === :light_mode && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1968))
    f === :light_bodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1976))
    f === :light_targetbodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 1984))
    f === :light_directional && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 1992))
    f === :light_castshadow && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2000))
    f === :light_active && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2008))
    f === :light_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2016))
    f === :light_dir && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2024))
    f === :light_poscom0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2032))
    f === :light_pos0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2040))
    f === :light_dir0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2048))
    f === :light_attenuation && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2056))
    f === :light_cutoff && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2064))
    f === :light_exponent && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2072))
    f === :light_ambient && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2080))
    f === :light_diffuse && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2088))
    f === :light_specular && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2096))
    f === :mesh_vertadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2104))
    f === :mesh_vertnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2112))
    f === :mesh_faceadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2120))
    f === :mesh_facenum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2128))
    f === :mesh_bvhadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2136))
    f === :mesh_bvhnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2144))
    f === :mesh_normaladr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2152))
    f === :mesh_normalnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2160))
    f === :mesh_texcoordadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2168))
    f === :mesh_texcoordnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2176))
    f === :mesh_graphadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2184))
    f === :mesh_vert && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2192))
    f === :mesh_normal && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2200))
    f === :mesh_texcoord && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2208))
    f === :mesh_face && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2216))
    f === :mesh_facenormal && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2224))
    f === :mesh_facetexcoord && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2232))
    f === :mesh_graph && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2240))
    f === :skin_matid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2248))
    f === :skin_group && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2256))
    f === :skin_rgba && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2264))
    f === :skin_inflate && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2272))
    f === :skin_vertadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2280))
    f === :skin_vertnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2288))
    f === :skin_texcoordadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2296))
    f === :skin_faceadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2304))
    f === :skin_facenum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2312))
    f === :skin_boneadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2320))
    f === :skin_bonenum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2328))
    f === :skin_vert && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2336))
    f === :skin_texcoord && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2344))
    f === :skin_face && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2352))
    f === :skin_bonevertadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2360))
    f === :skin_bonevertnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2368))
    f === :skin_bonebindpos && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2376))
    f === :skin_bonebindquat && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2384))
    f === :skin_bonebodyid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2392))
    f === :skin_bonevertid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2400))
    f === :skin_bonevertweight && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2408))
    f === :hfield_size && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2416))
    f === :hfield_nrow && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2424))
    f === :hfield_ncol && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2432))
    f === :hfield_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2440))
    f === :hfield_data && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2448))
    f === :tex_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2456))
    f === :tex_height && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2464))
    f === :tex_width && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2472))
    f === :tex_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2480))
    f === :tex_rgb && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2488))
    f === :mat_texid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2496))
    f === :mat_texuniform && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2504))
    f === :mat_texrepeat && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2512))
    f === :mat_emission && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2520))
    f === :mat_specular && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2528))
    f === :mat_shininess && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2536))
    f === :mat_reflectance && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2544))
    f === :mat_rgba && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2552))
    f === :pair_dim && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2560))
    f === :pair_geom1 && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2568))
    f === :pair_geom2 && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2576))
    f === :pair_signature && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2584))
    f === :pair_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2592))
    f === :pair_solreffriction && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2600))
    f === :pair_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2608))
    f === :pair_margin && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2616))
    f === :pair_gap && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2624))
    f === :pair_friction && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2632))
    f === :exclude_signature && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2640))
    f === :eq_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2648))
    f === :eq_obj1id && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2656))
    f === :eq_obj2id && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2664))
    f === :eq_active && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2672))
    f === :eq_solref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2680))
    f === :eq_solimp && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2688))
    f === :eq_data && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2696))
    f === :tendon_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2704))
    f === :tendon_num && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2712))
    f === :tendon_matid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2720))
    f === :tendon_group && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2728))
    f === :tendon_limited && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2736))
    f === :tendon_width && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2744))
    f === :tendon_solref_lim && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2752))
    f === :tendon_solimp_lim && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2760))
    f === :tendon_solref_fri && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2768))
    f === :tendon_solimp_fri && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2776))
    f === :tendon_range && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2784))
    f === :tendon_margin && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2792))
    f === :tendon_stiffness && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2800))
    f === :tendon_damping && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2808))
    f === :tendon_frictionloss && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2816))
    f === :tendon_lengthspring && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2824))
    f === :tendon_length0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2832))
    f === :tendon_invweight0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2840))
    f === :tendon_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2848))
    f === :tendon_rgba && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 2856))
    f === :wrap_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2864))
    f === :wrap_objid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2872))
    f === :wrap_prm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2880))
    f === :actuator_trntype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2888))
    f === :actuator_dyntype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2896))
    f === :actuator_gaintype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2904))
    f === :actuator_biastype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2912))
    f === :actuator_trnid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2920))
    f === :actuator_actadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2928))
    f === :actuator_actnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2936))
    f === :actuator_group && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 2944))
    f === :actuator_ctrllimited && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2952))
    f === :actuator_forcelimited && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2960))
    f === :actuator_actlimited && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 2968))
    f === :actuator_dynprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2976))
    f === :actuator_gainprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2984))
    f === :actuator_biasprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 2992))
    f === :actuator_ctrlrange && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3000))
    f === :actuator_forcerange && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3008))
    f === :actuator_actrange && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3016))
    f === :actuator_gear && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3024))
    f === :actuator_cranklength && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3032))
    f === :actuator_acc0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3040))
    f === :actuator_length0 && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3048))
    f === :actuator_lengthrange && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3056))
    f === :actuator_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3064))
    f === :actuator_plugin && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3072))
    f === :sensor_type && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3080))
    f === :sensor_datatype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3088))
    f === :sensor_needstage && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3096))
    f === :sensor_objtype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3104))
    f === :sensor_objid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3112))
    f === :sensor_reftype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3120))
    f === :sensor_refid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3128))
    f === :sensor_dim && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3136))
    f === :sensor_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3144))
    f === :sensor_cutoff && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3152))
    f === :sensor_noise && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3160))
    f === :sensor_user && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3168))
    f === :sensor_plugin && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3176))
    f === :plugin && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3184))
    f === :plugin_stateadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3192))
    f === :plugin_statenum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3200))
    f === :plugin_attr && return unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3208))
    f === :plugin_attradr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3216))
    f === :numeric_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3224))
    f === :numeric_size && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3232))
    f === :numeric_data && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3240))
    f === :text_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3248))
    f === :text_size && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3256))
    f === :text_data && return unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3264))
    f === :tuple_adr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3272))
    f === :tuple_size && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3280))
    f === :tuple_objtype && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3288))
    f === :tuple_objid && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3296))
    f === :tuple_objprm && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3304))
    f === :key_time && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3312))
    f === :key_qpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3320))
    f === :key_qvel && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3328))
    f === :key_act && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3336))
    f === :key_mpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3344))
    f === :key_mquat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3352))
    f === :key_ctrl && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 3360))
    f === :name_bodyadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3368))
    f === :name_jntadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3376))
    f === :name_geomadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3384))
    f === :name_siteadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3392))
    f === :name_camadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3400))
    f === :name_lightadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3408))
    f === :name_meshadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3416))
    f === :name_skinadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3424))
    f === :name_hfieldadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3432))
    f === :name_texadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3440))
    f === :name_matadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3448))
    f === :name_pairadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3456))
    f === :name_excludeadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3464))
    f === :name_eqadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3472))
    f === :name_tendonadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3480))
    f === :name_actuatoradr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3488))
    f === :name_sensoradr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3496))
    f === :name_numericadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3504))
    f === :name_textadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3512))
    f === :name_tupleadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3520))
    f === :name_keyadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3528))
    f === :name_pluginadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3536))
    f === :names && return unsafe_load(Ptr{Ptr{Int8}}(internal_pointer + 3544))
    f === :names_map && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 3552))
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
    f === :qpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40432))
    f === :qvel && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40440))
    f === :act && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40448))
    f === :qacc_warmstart && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40456))
    f === :plugin_state && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40464))
    f === :ctrl && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40472))
    f === :qfrc_applied && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40480))
    f === :xfrc_applied && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40488))
    f === :mocap_pos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40496))
    f === :mocap_quat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40504))
    f === :qacc && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40512))
    f === :act_dot && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40520))
    f === :userdata && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40528))
    f === :sensordata && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40536))
    f === :plugin && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40544))
    f === :plugin_data && return unsafe_load(Ptr{Ptr{UInt64}}(internal_pointer + 40552))
    f === :xpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40560))
    f === :xquat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40568))
    f === :xmat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40576))
    f === :xipos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40584))
    f === :ximat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40592))
    f === :xanchor && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40600))
    f === :xaxis && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40608))
    f === :geom_xpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40616))
    f === :geom_xmat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40624))
    f === :site_xpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40632))
    f === :site_xmat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40640))
    f === :cam_xpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40648))
    f === :cam_xmat && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40656))
    f === :light_xpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40664))
    f === :light_xdir && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40672))
    f === :subtree_com && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40680))
    f === :cdof && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40688))
    f === :cinert && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40696))
    f === :ten_wrapadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40704))
    f === :ten_wrapnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40712))
    f === :ten_J_rownnz && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40720))
    f === :ten_J_rowadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40728))
    f === :ten_J_colind && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40736))
    f === :ten_length && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40744))
    f === :ten_J && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40752))
    f === :wrap_obj && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40760))
    f === :wrap_xpos && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40768))
    f === :actuator_length && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40776))
    f === :actuator_moment && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40784))
    f === :crb && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40792))
    f === :qM && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40800))
    f === :qLD && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40808))
    f === :qLDiagInv && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40816))
    f === :qLDiagSqrtInv && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40824))
    f === :bvh_active && return unsafe_load(Ptr{Ptr{UInt8}}(internal_pointer + 40832))
    f === :ten_velocity && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40840))
    f === :actuator_velocity && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40848))
    f === :cvel && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40856))
    f === :cdof_dot && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40864))
    f === :qfrc_bias && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40872))
    f === :qfrc_passive && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40880))
    f === :efc_vel && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40888))
    f === :efc_aref && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40896))
    f === :subtree_linvel && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40904))
    f === :subtree_angmom && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40912))
    f === :qH && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40920))
    f === :qHDiagInv && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40928))
    f === :D_rownnz && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40936))
    f === :D_rowadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40944))
    f === :D_colind && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40952))
    f === :B_rownnz && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40960))
    f === :B_rowadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40968))
    f === :B_colind && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40976))
    f === :qDeriv && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40984))
    f === :qLU && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 40992))
    f === :actuator_force && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41000))
    f === :qfrc_actuator && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41008))
    f === :qfrc_smooth && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41016))
    f === :qacc_smooth && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41024))
    f === :qfrc_constraint && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41032))
    f === :qfrc_inverse && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41040))
    f === :cacc && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41048))
    f === :cfrc_int && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41056))
    f === :cfrc_ext && return unsafe_load(Ptr{Ptr{Float64}}(internal_pointer + 41064))
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
