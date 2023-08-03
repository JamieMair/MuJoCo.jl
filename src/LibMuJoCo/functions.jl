function Base.getproperty(x::Ptr{var"##Ctag#314"}, f::Symbol)
    f === :fovy && return Ptr{Cfloat}(x + 0)
    f === :ipd && return Ptr{Cfloat}(x + 4)
    f === :azimuth && return Ptr{Cfloat}(x + 8)
    f === :elevation && return Ptr{Cfloat}(x + 12)
    f === :linewidth && return Ptr{Cfloat}(x + 16)
    f === :glow && return Ptr{Cfloat}(x + 20)
    f === :realtime && return Ptr{Cfloat}(x + 24)
    f === :offwidth && return Ptr{Cint}(x + 28)
    f === :offheight && return Ptr{Cint}(x + 32)
    f === :ellipsoidinertia && return Ptr{Cint}(x + 36)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#314", f::Symbol)
    r = Ref{var"##Ctag#314"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#314"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#314"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#315"}, f::Symbol)
    f === :shadowsize && return Ptr{Cint}(x + 0)
    f === :offsamples && return Ptr{Cint}(x + 4)
    f === :numslices && return Ptr{Cint}(x + 8)
    f === :numstacks && return Ptr{Cint}(x + 12)
    f === :numquads && return Ptr{Cint}(x + 16)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#315", f::Symbol)
    r = Ref{var"##Ctag#315"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#315"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#315"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#316"}, f::Symbol)
    f === :ambient && return Ptr{NTuple{3, Cfloat}}(x + 0)
    f === :diffuse && return Ptr{NTuple{3, Cfloat}}(x + 12)
    f === :specular && return Ptr{NTuple{3, Cfloat}}(x + 24)
    f === :active && return Ptr{Cint}(x + 36)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#316", f::Symbol)
    r = Ref{var"##Ctag#316"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#316"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#316"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#317"}, f::Symbol)
    f === :stiffness && return Ptr{Cfloat}(x + 0)
    f === :stiffnessrot && return Ptr{Cfloat}(x + 4)
    f === :force && return Ptr{Cfloat}(x + 8)
    f === :torque && return Ptr{Cfloat}(x + 12)
    f === :alpha && return Ptr{Cfloat}(x + 16)
    f === :fogstart && return Ptr{Cfloat}(x + 20)
    f === :fogend && return Ptr{Cfloat}(x + 24)
    f === :znear && return Ptr{Cfloat}(x + 28)
    f === :zfar && return Ptr{Cfloat}(x + 32)
    f === :haze && return Ptr{Cfloat}(x + 36)
    f === :shadowclip && return Ptr{Cfloat}(x + 40)
    f === :shadowscale && return Ptr{Cfloat}(x + 44)
    f === :actuatortendon && return Ptr{Cfloat}(x + 48)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#317", f::Symbol)
    r = Ref{var"##Ctag#317"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#317"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#317"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#318"}, f::Symbol)
    f === :forcewidth && return Ptr{Cfloat}(x + 0)
    f === :contactwidth && return Ptr{Cfloat}(x + 4)
    f === :contactheight && return Ptr{Cfloat}(x + 8)
    f === :connect && return Ptr{Cfloat}(x + 12)
    f === :com && return Ptr{Cfloat}(x + 16)
    f === :camera && return Ptr{Cfloat}(x + 20)
    f === :light && return Ptr{Cfloat}(x + 24)
    f === :selectpoint && return Ptr{Cfloat}(x + 28)
    f === :jointlength && return Ptr{Cfloat}(x + 32)
    f === :jointwidth && return Ptr{Cfloat}(x + 36)
    f === :actuatorlength && return Ptr{Cfloat}(x + 40)
    f === :actuatorwidth && return Ptr{Cfloat}(x + 44)
    f === :framelength && return Ptr{Cfloat}(x + 48)
    f === :framewidth && return Ptr{Cfloat}(x + 52)
    f === :constraint && return Ptr{Cfloat}(x + 56)
    f === :slidercrank && return Ptr{Cfloat}(x + 60)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#318", f::Symbol)
    r = Ref{var"##Ctag#318"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#318"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#318"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#319"}, f::Symbol)
    f === :fog && return Ptr{NTuple{4, Cfloat}}(x + 0)
    f === :haze && return Ptr{NTuple{4, Cfloat}}(x + 16)
    f === :force && return Ptr{NTuple{4, Cfloat}}(x + 32)
    f === :inertia && return Ptr{NTuple{4, Cfloat}}(x + 48)
    f === :joint && return Ptr{NTuple{4, Cfloat}}(x + 64)
    f === :actuator && return Ptr{NTuple{4, Cfloat}}(x + 80)
    f === :actuatornegative && return Ptr{NTuple{4, Cfloat}}(x + 96)
    f === :actuatorpositive && return Ptr{NTuple{4, Cfloat}}(x + 112)
    f === :com && return Ptr{NTuple{4, Cfloat}}(x + 128)
    f === :camera && return Ptr{NTuple{4, Cfloat}}(x + 144)
    f === :light && return Ptr{NTuple{4, Cfloat}}(x + 160)
    f === :selectpoint && return Ptr{NTuple{4, Cfloat}}(x + 176)
    f === :connect && return Ptr{NTuple{4, Cfloat}}(x + 192)
    f === :contactpoint && return Ptr{NTuple{4, Cfloat}}(x + 208)
    f === :contactforce && return Ptr{NTuple{4, Cfloat}}(x + 224)
    f === :contactfriction && return Ptr{NTuple{4, Cfloat}}(x + 240)
    f === :contacttorque && return Ptr{NTuple{4, Cfloat}}(x + 256)
    f === :contactgap && return Ptr{NTuple{4, Cfloat}}(x + 272)
    f === :rangefinder && return Ptr{NTuple{4, Cfloat}}(x + 288)
    f === :constraint && return Ptr{NTuple{4, Cfloat}}(x + 304)
    f === :slidercrank && return Ptr{NTuple{4, Cfloat}}(x + 320)
    f === :crankbroken && return Ptr{NTuple{4, Cfloat}}(x + 336)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#319", f::Symbol)
    r = Ref{var"##Ctag#319"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#319"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#319"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{mjVisual_}, f::Symbol)
    f === :_global && return Ptr{var"##Ctag#314"}(x + 0)
    f === :quality && return Ptr{var"##Ctag#315"}(x + 40)
    f === :headlight && return Ptr{var"##Ctag#316"}(x + 60)
    f === :map && return Ptr{var"##Ctag#317"}(x + 100)
    f === :scale && return Ptr{var"##Ctag#318"}(x + 152)
    f === :rgba && return Ptr{var"##Ctag#319"}(x + 216)
    return getfield(x, f)
end
function Base.getproperty(x::mjVisual_, f::Symbol)
    r = Ref{mjVisual_}(x)
    ptr = Base.unsafe_convert(Ptr{mjVisual_}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{mjVisual_}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{mjuiItem_}, f::Symbol)
    f === :type && return Ptr{Cint}(x + 0)
    f === :name && return Ptr{NTuple{40, Cchar}}(x + 4)
    f === :state && return Ptr{Cint}(x + 44)
    f === :pdata && return Ptr{Ptr{Cvoid}}(x + 48)
    f === :sectionid && return Ptr{Cint}(x + 56)
    f === :itemid && return Ptr{Cint}(x + 60)
    f === :single && return Ptr{mjuiItemSingle_}(x + 64)
    f === :multi && return Ptr{mjuiItemMulti_}(x + 64)
    f === :slider && return Ptr{mjuiItemSlider_}(x + 64)
    f === :edit && return Ptr{mjuiItemEdit_}(x + 64)
    f === :rect && return Ptr{mjrRect}(x + 1472)
    return getfield(x, f)
end
function Base.getproperty(x::mjuiItem_, f::Symbol)
    r = Ref{mjuiItem_}(x)
    ptr = Base.unsafe_convert(Ptr{mjuiItem_}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{mjuiItem_}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#320"}, f::Symbol)
    f === :nu && return Ptr{Cint}(x + 0)
    f === :na && return Ptr{Cint}(x + 4)
    f === :nbody && return Ptr{Cint}(x + 8)
    f === :nbvh && return Ptr{Cint}(x + 12)
    f === :njnt && return Ptr{Cint}(x + 16)
    f === :ngeom && return Ptr{Cint}(x + 20)
    f === :nsite && return Ptr{Cint}(x + 24)
    f === :ncam && return Ptr{Cint}(x + 28)
    f === :nlight && return Ptr{Cint}(x + 32)
    f === :nmesh && return Ptr{Cint}(x + 36)
    f === :nskin && return Ptr{Cint}(x + 40)
    f === :nskinvert && return Ptr{Cint}(x + 44)
    f === :nskinface && return Ptr{Cint}(x + 48)
    f === :nskinbone && return Ptr{Cint}(x + 52)
    f === :nskinbonevert && return Ptr{Cint}(x + 56)
    f === :nmat && return Ptr{Cint}(x + 60)
    f === :neq && return Ptr{Cint}(x + 64)
    f === :ntendon && return Ptr{Cint}(x + 68)
    f === :nwrap && return Ptr{Cint}(x + 72)
    f === :nsensor && return Ptr{Cint}(x + 76)
    f === :nnames && return Ptr{Cint}(x + 80)
    f === :nsensordata && return Ptr{Cint}(x + 84)
    f === :opt && return Ptr{mjOption}(x + 88)
    f === :vis && return Ptr{mjVisual}(x + 328)
    f === :stat && return Ptr{mjStatistic}(x + 896)
    f === :body_parentid && return Ptr{Ptr{Cint}}(x + 952)
    f === :body_rootid && return Ptr{Ptr{Cint}}(x + 960)
    f === :body_weldid && return Ptr{Ptr{Cint}}(x + 968)
    f === :body_mocapid && return Ptr{Ptr{Cint}}(x + 976)
    f === :body_jntnum && return Ptr{Ptr{Cint}}(x + 984)
    f === :body_jntadr && return Ptr{Ptr{Cint}}(x + 992)
    f === :body_geomnum && return Ptr{Ptr{Cint}}(x + 1000)
    f === :body_geomadr && return Ptr{Ptr{Cint}}(x + 1008)
    f === :body_iquat && return Ptr{Ptr{mjtNum}}(x + 1016)
    f === :body_mass && return Ptr{Ptr{mjtNum}}(x + 1024)
    f === :body_inertia && return Ptr{Ptr{mjtNum}}(x + 1032)
    f === :body_bvhadr && return Ptr{Ptr{Cint}}(x + 1040)
    f === :body_bvhnum && return Ptr{Ptr{Cint}}(x + 1048)
    f === :bvh_depth && return Ptr{Ptr{Cint}}(x + 1056)
    f === :bvh_child && return Ptr{Ptr{Cint}}(x + 1064)
    f === :bvh_geomid && return Ptr{Ptr{Cint}}(x + 1072)
    f === :bvh_aabb && return Ptr{Ptr{mjtNum}}(x + 1080)
    f === :jnt_type && return Ptr{Ptr{Cint}}(x + 1088)
    f === :jnt_bodyid && return Ptr{Ptr{Cint}}(x + 1096)
    f === :jnt_group && return Ptr{Ptr{Cint}}(x + 1104)
    f === :geom_type && return Ptr{Ptr{Cint}}(x + 1112)
    f === :geom_bodyid && return Ptr{Ptr{Cint}}(x + 1120)
    f === :geom_dataid && return Ptr{Ptr{Cint}}(x + 1128)
    f === :geom_matid && return Ptr{Ptr{Cint}}(x + 1136)
    f === :geom_group && return Ptr{Ptr{Cint}}(x + 1144)
    f === :geom_size && return Ptr{Ptr{mjtNum}}(x + 1152)
    f === :geom_aabb && return Ptr{Ptr{mjtNum}}(x + 1160)
    f === :geom_rbound && return Ptr{Ptr{mjtNum}}(x + 1168)
    f === :geom_rgba && return Ptr{Ptr{Cfloat}}(x + 1176)
    f === :site_type && return Ptr{Ptr{Cint}}(x + 1184)
    f === :site_bodyid && return Ptr{Ptr{Cint}}(x + 1192)
    f === :site_matid && return Ptr{Ptr{Cint}}(x + 1200)
    f === :site_group && return Ptr{Ptr{Cint}}(x + 1208)
    f === :site_size && return Ptr{Ptr{mjtNum}}(x + 1216)
    f === :site_rgba && return Ptr{Ptr{Cfloat}}(x + 1224)
    f === :cam_fovy && return Ptr{Ptr{mjtNum}}(x + 1232)
    f === :cam_ipd && return Ptr{Ptr{mjtNum}}(x + 1240)
    f === :light_directional && return Ptr{Ptr{mjtByte}}(x + 1248)
    f === :light_castshadow && return Ptr{Ptr{mjtByte}}(x + 1256)
    f === :light_active && return Ptr{Ptr{mjtByte}}(x + 1264)
    f === :light_attenuation && return Ptr{Ptr{Cfloat}}(x + 1272)
    f === :light_cutoff && return Ptr{Ptr{Cfloat}}(x + 1280)
    f === :light_exponent && return Ptr{Ptr{Cfloat}}(x + 1288)
    f === :light_ambient && return Ptr{Ptr{Cfloat}}(x + 1296)
    f === :light_diffuse && return Ptr{Ptr{Cfloat}}(x + 1304)
    f === :light_specular && return Ptr{Ptr{Cfloat}}(x + 1312)
    f === :mesh_texcoordadr && return Ptr{Ptr{Cint}}(x + 1320)
    f === :mesh_graphadr && return Ptr{Ptr{Cint}}(x + 1328)
    f === :skin_matid && return Ptr{Ptr{Cint}}(x + 1336)
    f === :skin_group && return Ptr{Ptr{Cint}}(x + 1344)
    f === :skin_rgba && return Ptr{Ptr{Cfloat}}(x + 1352)
    f === :skin_inflate && return Ptr{Ptr{Cfloat}}(x + 1360)
    f === :skin_vertadr && return Ptr{Ptr{Cint}}(x + 1368)
    f === :skin_vertnum && return Ptr{Ptr{Cint}}(x + 1376)
    f === :skin_texcoordadr && return Ptr{Ptr{Cint}}(x + 1384)
    f === :skin_faceadr && return Ptr{Ptr{Cint}}(x + 1392)
    f === :skin_facenum && return Ptr{Ptr{Cint}}(x + 1400)
    f === :skin_boneadr && return Ptr{Ptr{Cint}}(x + 1408)
    f === :skin_bonenum && return Ptr{Ptr{Cint}}(x + 1416)
    f === :skin_vert && return Ptr{Ptr{Cfloat}}(x + 1424)
    f === :skin_face && return Ptr{Ptr{Cint}}(x + 1432)
    f === :skin_bonevertadr && return Ptr{Ptr{Cint}}(x + 1440)
    f === :skin_bonevertnum && return Ptr{Ptr{Cint}}(x + 1448)
    f === :skin_bonebindpos && return Ptr{Ptr{Cfloat}}(x + 1456)
    f === :skin_bonebindquat && return Ptr{Ptr{Cfloat}}(x + 1464)
    f === :skin_bonebodyid && return Ptr{Ptr{Cint}}(x + 1472)
    f === :skin_bonevertid && return Ptr{Ptr{Cint}}(x + 1480)
    f === :skin_bonevertweight && return Ptr{Ptr{Cfloat}}(x + 1488)
    f === :mat_texid && return Ptr{Ptr{Cint}}(x + 1496)
    f === :mat_texuniform && return Ptr{Ptr{mjtByte}}(x + 1504)
    f === :mat_texrepeat && return Ptr{Ptr{Cfloat}}(x + 1512)
    f === :mat_emission && return Ptr{Ptr{Cfloat}}(x + 1520)
    f === :mat_specular && return Ptr{Ptr{Cfloat}}(x + 1528)
    f === :mat_shininess && return Ptr{Ptr{Cfloat}}(x + 1536)
    f === :mat_reflectance && return Ptr{Ptr{Cfloat}}(x + 1544)
    f === :mat_rgba && return Ptr{Ptr{Cfloat}}(x + 1552)
    f === :eq_type && return Ptr{Ptr{Cint}}(x + 1560)
    f === :eq_obj1id && return Ptr{Ptr{Cint}}(x + 1568)
    f === :eq_obj2id && return Ptr{Ptr{Cint}}(x + 1576)
    f === :eq_active && return Ptr{Ptr{mjtByte}}(x + 1584)
    f === :eq_data && return Ptr{Ptr{mjtNum}}(x + 1592)
    f === :tendon_num && return Ptr{Ptr{Cint}}(x + 1600)
    f === :tendon_matid && return Ptr{Ptr{Cint}}(x + 1608)
    f === :tendon_group && return Ptr{Ptr{Cint}}(x + 1616)
    f === :tendon_limited && return Ptr{Ptr{mjtByte}}(x + 1624)
    f === :tendon_width && return Ptr{Ptr{mjtNum}}(x + 1632)
    f === :tendon_range && return Ptr{Ptr{mjtNum}}(x + 1640)
    f === :tendon_stiffness && return Ptr{Ptr{mjtNum}}(x + 1648)
    f === :tendon_damping && return Ptr{Ptr{mjtNum}}(x + 1656)
    f === :tendon_frictionloss && return Ptr{Ptr{mjtNum}}(x + 1664)
    f === :tendon_lengthspring && return Ptr{Ptr{mjtNum}}(x + 1672)
    f === :tendon_rgba && return Ptr{Ptr{Cfloat}}(x + 1680)
    f === :actuator_trntype && return Ptr{Ptr{Cint}}(x + 1688)
    f === :actuator_dyntype && return Ptr{Ptr{Cint}}(x + 1696)
    f === :actuator_trnid && return Ptr{Ptr{Cint}}(x + 1704)
    f === :actuator_actadr && return Ptr{Ptr{Cint}}(x + 1712)
    f === :actuator_actnum && return Ptr{Ptr{Cint}}(x + 1720)
    f === :actuator_group && return Ptr{Ptr{Cint}}(x + 1728)
    f === :actuator_ctrllimited && return Ptr{Ptr{mjtByte}}(x + 1736)
    f === :actuator_actlimited && return Ptr{Ptr{mjtByte}}(x + 1744)
    f === :actuator_ctrlrange && return Ptr{Ptr{mjtNum}}(x + 1752)
    f === :actuator_actrange && return Ptr{Ptr{mjtNum}}(x + 1760)
    f === :actuator_cranklength && return Ptr{Ptr{mjtNum}}(x + 1768)
    f === :sensor_type && return Ptr{Ptr{Cint}}(x + 1776)
    f === :sensor_objid && return Ptr{Ptr{Cint}}(x + 1784)
    f === :sensor_adr && return Ptr{Ptr{Cint}}(x + 1792)
    f === :name_bodyadr && return Ptr{Ptr{Cint}}(x + 1800)
    f === :name_jntadr && return Ptr{Ptr{Cint}}(x + 1808)
    f === :name_geomadr && return Ptr{Ptr{Cint}}(x + 1816)
    f === :name_siteadr && return Ptr{Ptr{Cint}}(x + 1824)
    f === :name_camadr && return Ptr{Ptr{Cint}}(x + 1832)
    f === :name_lightadr && return Ptr{Ptr{Cint}}(x + 1840)
    f === :name_eqadr && return Ptr{Ptr{Cint}}(x + 1848)
    f === :name_tendonadr && return Ptr{Ptr{Cint}}(x + 1856)
    f === :name_actuatoradr && return Ptr{Ptr{Cint}}(x + 1864)
    f === :names && return Ptr{Ptr{Cchar}}(x + 1872)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#320", f::Symbol)
    r = Ref{var"##Ctag#320"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#320"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#320"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#321"}, f::Symbol)
    f === :warning && return Ptr{NTuple{8, mjWarningStat}}(x + 0)
    f === :nefc && return Ptr{Cint}(x + 64)
    f === :ncon && return Ptr{Cint}(x + 68)
    f === :time && return Ptr{mjtNum}(x + 72)
    f === :act && return Ptr{Ptr{mjtNum}}(x + 80)
    f === :ctrl && return Ptr{Ptr{mjtNum}}(x + 88)
    f === :xfrc_applied && return Ptr{Ptr{mjtNum}}(x + 96)
    f === :sensordata && return Ptr{Ptr{mjtNum}}(x + 104)
    f === :xpos && return Ptr{Ptr{mjtNum}}(x + 112)
    f === :xquat && return Ptr{Ptr{mjtNum}}(x + 120)
    f === :xmat && return Ptr{Ptr{mjtNum}}(x + 128)
    f === :xipos && return Ptr{Ptr{mjtNum}}(x + 136)
    f === :ximat && return Ptr{Ptr{mjtNum}}(x + 144)
    f === :xanchor && return Ptr{Ptr{mjtNum}}(x + 152)
    f === :xaxis && return Ptr{Ptr{mjtNum}}(x + 160)
    f === :geom_xpos && return Ptr{Ptr{mjtNum}}(x + 168)
    f === :geom_xmat && return Ptr{Ptr{mjtNum}}(x + 176)
    f === :site_xpos && return Ptr{Ptr{mjtNum}}(x + 184)
    f === :site_xmat && return Ptr{Ptr{mjtNum}}(x + 192)
    f === :cam_xpos && return Ptr{Ptr{mjtNum}}(x + 200)
    f === :cam_xmat && return Ptr{Ptr{mjtNum}}(x + 208)
    f === :light_xpos && return Ptr{Ptr{mjtNum}}(x + 216)
    f === :light_xdir && return Ptr{Ptr{mjtNum}}(x + 224)
    f === :subtree_com && return Ptr{Ptr{mjtNum}}(x + 232)
    f === :ten_wrapadr && return Ptr{Ptr{Cint}}(x + 240)
    f === :ten_wrapnum && return Ptr{Ptr{Cint}}(x + 248)
    f === :wrap_obj && return Ptr{Ptr{Cint}}(x + 256)
    f === :wrap_xpos && return Ptr{Ptr{mjtNum}}(x + 264)
    f === :bvh_active && return Ptr{Ptr{mjtByte}}(x + 272)
    f === :contact && return Ptr{Ptr{mjContact}}(x + 280)
    f === :efc_force && return Ptr{Ptr{mjtNum}}(x + 288)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#321", f::Symbol)
    r = Ref{var"##Ctag#321"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#321"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#321"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{mjvSceneState_}, f::Symbol)
    f === :nbuffer && return Ptr{Cint}(x + 0)
    f === :buffer && return Ptr{Ptr{Cvoid}}(x + 8)
    f === :maxgeom && return Ptr{Cint}(x + 16)
    f === :plugincache && return Ptr{mjvScene}(x + 24)
    f === :model && return Ptr{var"##Ctag#320"}(x + 8680)
    f === :data && return Ptr{var"##Ctag#321"}(x + 10560)
    return getfield(x, f)
end
function Base.getproperty(x::mjvSceneState_, f::Symbol)
    r = Ref{mjvSceneState_}(x)
    ptr = Base.unsafe_convert(Ptr{mjvSceneState_}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{mjvSceneState_}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function mj_defaultVFS(vfs)
    ccall((:mj_defaultVFS, libmujuco), Cvoid, (Ptr{mjVFS},), vfs)
end
function mj_addFileVFS(vfs, directory, filename)
    ccall((:mj_addFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}, Ptr{Cchar}), vfs, directory, filename)
end
function mj_makeEmptyFileVFS(vfs, filename, filesize)
    ccall((:mj_makeEmptyFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}, Cint), vfs, filename, filesize)
end
function mj_findFileVFS(vfs, filename)
    ccall((:mj_findFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}), vfs, filename)
end
function mj_deleteFileVFS(vfs, filename)
    ccall((:mj_deleteFileVFS, libmujuco), Cint, (Ptr{mjVFS}, Ptr{Cchar}), vfs, filename)
end
function mj_deleteVFS(vfs)
    ccall((:mj_deleteVFS, libmujuco), Cvoid, (Ptr{mjVFS},), vfs)
end
function mj_loadXML(filename, vfs, error, error_sz)
    ccall((:mj_loadXML, libmujuco), Ptr{mjModel}, (Ptr{Cchar}, Ptr{mjVFS}, Ptr{Cchar}, Cint), filename, vfs, error, error_sz)
end
function mj_saveLastXML(filename, m, error, error_sz)
    ccall((:mj_saveLastXML, libmujuco), Cint, (Ptr{Cchar}, Ptr{mjModel}, Ptr{Cchar}, Cint), filename, m, error, error_sz)
end
function mj_freeLastXML()
    ccall((:mj_freeLastXML, libmujuco), Cvoid, ())
end
function mj_printSchema(filename, buffer, buffer_sz, flg_html, flg_pad)
    ccall((:mj_printSchema, libmujuco), Cint, (Ptr{Cchar}, Ptr{Cchar}, Cint, Cint, Cint), filename, buffer, buffer_sz, flg_html, flg_pad)
end
function mj_step(m, d)
    ccall((:mj_step, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_step1(m, d)
    ccall((:mj_step1, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_step2(m, d)
    ccall((:mj_step2, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_forward(m, d)
    ccall((:mj_forward, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_inverse(m, d)
    ccall((:mj_inverse, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_forwardSkip(m, d, skipstage, skipsensor)
    ccall((:mj_forwardSkip, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint), m, d, skipstage, skipsensor)
end
function mj_inverseSkip(m, d, skipstage, skipsensor)
    ccall((:mj_inverseSkip, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint), m, d, skipstage, skipsensor)
end
function mj_defaultLROpt(opt)
    ccall((:mj_defaultLROpt, libmujuco), Cvoid, (Ptr{mjLROpt},), opt)
end
function mj_defaultSolRefImp(solref, solimp)
    ccall((:mj_defaultSolRefImp, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), solref, solimp)
end
function mj_defaultOption(opt)
    ccall((:mj_defaultOption, libmujuco), Cvoid, (Ptr{mjOption},), opt)
end
function mj_defaultVisual(vis)
    ccall((:mj_defaultVisual, libmujuco), Cvoid, (Ptr{mjVisual},), vis)
end
function mj_copyModel(dest, src)
    ccall((:mj_copyModel, libmujuco), Ptr{mjModel}, (Ptr{mjModel}, Ptr{mjModel}), dest, src)
end
function mj_saveModel(m, filename, buffer, buffer_sz)
    ccall((:mj_saveModel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}, Ptr{Cvoid}, Cint), m, filename, buffer, buffer_sz)
end
function mj_loadModel(filename, vfs)
    ccall((:mj_loadModel, libmujuco), Ptr{mjModel}, (Ptr{Cchar}, Ptr{mjVFS}), filename, vfs)
end
function mj_deleteModel(m)
    ccall((:mj_deleteModel, libmujuco), Cvoid, (Ptr{mjModel},), m)
end
function mj_sizeModel(m)
    ccall((:mj_sizeModel, libmujuco), Cint, (Ptr{mjModel},), m)
end
function mj_makeData(m)
    ccall((:mj_makeData, libmujuco), Ptr{mjData}, (Ptr{mjModel},), m)
end
function mj_copyData(dest, m, src)
    ccall((:mj_copyData, libmujuco), Ptr{mjData}, (Ptr{mjData}, Ptr{mjModel}, Ptr{mjData}), dest, m, src)
end
function mj_resetData(m, d)
    ccall((:mj_resetData, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_resetDataDebug(m, d, debug_value)
    ccall((:mj_resetDataDebug, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cuchar), m, d, debug_value)
end
function mj_resetDataKeyframe(m, d, key)
    ccall((:mj_resetDataKeyframe, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, key)
end
function mj_stackAlloc(d, size)
    ccall((:mj_stackAlloc, libmujuco), Ptr{mjtNum}, (Ptr{mjData}, Cint), d, size)
end
function mj_stackAllocInt(d, size)
    ccall((:mj_stackAllocInt, libmujuco), Ptr{Cint}, (Ptr{mjData}, Cint), d, size)
end
function mj_deleteData(d)
    ccall((:mj_deleteData, libmujuco), Cvoid, (Ptr{mjData},), d)
end
function mj_resetCallbacks()
    ccall((:mj_resetCallbacks, libmujuco), Cvoid, ())
end
function mj_setConst(m, d)
    ccall((:mj_setConst, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_setLengthRange(m, d, index, opt, error, error_sz)
    ccall((:mj_setLengthRange, libmujuco), Cint, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjLROpt}, Ptr{Cchar}, Cint), m, d, index, opt, error, error_sz)
end
function mj_printFormattedModel(m, filename, float_format)
    ccall((:mj_printFormattedModel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}, Ptr{Cchar}), m, filename, float_format)
end
function mj_printModel(m, filename)
    ccall((:mj_printModel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}), m, filename)
end
function mj_printFormattedData(m, d, filename, float_format)
    ccall((:mj_printFormattedData, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{Cchar}, Ptr{Cchar}), m, d, filename, float_format)
end
function mj_printData(m, d, filename)
    ccall((:mj_printData, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{Cchar}), m, d, filename)
end
function mju_printMat(mat, nr, nc)
    ccall((:mju_printMat, libmujuco), Cvoid, (Ptr{mjtNum}, Cint, Cint), mat, nr, nc)
end
function mju_printMatSparse(mat, nr, rownnz, rowadr, colind)
    ccall((:mju_printMatSparse, libmujuco), Cvoid, (Ptr{mjtNum}, Cint, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), mat, nr, rownnz, rowadr, colind)
end
function mj_fwdPosition(m, d)
    ccall((:mj_fwdPosition, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_fwdVelocity(m, d)
    ccall((:mj_fwdVelocity, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_fwdActuation(m, d)
    ccall((:mj_fwdActuation, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_fwdAcceleration(m, d)
    ccall((:mj_fwdAcceleration, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_fwdConstraint(m, d)
    ccall((:mj_fwdConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_Euler(m, d)
    ccall((:mj_Euler, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_RungeKutta(m, d, N)
    ccall((:mj_RungeKutta, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, N)
end
function mj_invPosition(m, d)
    ccall((:mj_invPosition, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_invVelocity(m, d)
    ccall((:mj_invVelocity, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_invConstraint(m, d)
    ccall((:mj_invConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_compareFwdInv(m, d)
    ccall((:mj_compareFwdInv, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_sensorPos(m, d)
    ccall((:mj_sensorPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_sensorVel(m, d)
    ccall((:mj_sensorVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_sensorAcc(m, d)
    ccall((:mj_sensorAcc, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_energyPos(m, d)
    ccall((:mj_energyPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_energyVel(m, d)
    ccall((:mj_energyVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_checkPos(m, d)
    ccall((:mj_checkPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_checkVel(m, d)
    ccall((:mj_checkVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_checkAcc(m, d)
    ccall((:mj_checkAcc, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_kinematics(m, d)
    ccall((:mj_kinematics, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_comPos(m, d)
    ccall((:mj_comPos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_camlight(m, d)
    ccall((:mj_camlight, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_tendon(m, d)
    ccall((:mj_tendon, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_transmission(m, d)
    ccall((:mj_transmission, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_crb(m, d)
    ccall((:mj_crb, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_factorM(m, d)
    ccall((:mj_factorM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_solveM(m, d, x, y, n)
    ccall((:mj_solveM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, x, y, n)
end
function mj_solveM2(m, d, x, y, n)
    ccall((:mj_solveM2, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, x, y, n)
end
function mj_comVel(m, d)
    ccall((:mj_comVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_passive(m, d)
    ccall((:mj_passive, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_subtreeVel(m, d)
    ccall((:mj_subtreeVel, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_rne(m, d, flg_acc, result)
    ccall((:mj_rne, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}), m, d, flg_acc, result)
end
function mj_rnePostConstraint(m, d)
    ccall((:mj_rnePostConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_collision(m, d)
    ccall((:mj_collision, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_makeConstraint(m, d)
    ccall((:mj_makeConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_projectConstraint(m, d)
    ccall((:mj_projectConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_referenceConstraint(m, d)
    ccall((:mj_referenceConstraint, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
function mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)
    ccall((:mj_constraintUpdate, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jar, cost, flg_coneHessian)
end
function mj_stateSize(m, spec)
    ccall((:mj_stateSize, libmujuco), Cint, (Ptr{mjModel}, Cuint), m, spec)
end
function mj_getState(m, d, state, spec)
    ccall((:mj_getState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cuint), m, d, state, spec)
end
function mj_setState(m, d, state, spec)
    ccall((:mj_setState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cuint), m, d, state, spec)
end
function mj_addContact(m, d, con)
    ccall((:mj_addContact, libmujuco), Cint, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjContact}), m, d, con)
end
function mj_isPyramidal(m)
    ccall((:mj_isPyramidal, libmujuco), Cint, (Ptr{mjModel},), m)
end
function mj_isSparse(m)
    ccall((:mj_isSparse, libmujuco), Cint, (Ptr{mjModel},), m)
end
function mj_isDual(m)
    ccall((:mj_isDual, libmujuco), Cint, (Ptr{mjModel},), m)
end
function mj_mulJacVec(m, d, res, vec)
    ccall((:mj_mulJacVec, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
function mj_mulJacTVec(m, d, res, vec)
    ccall((:mj_mulJacTVec, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
function mj_jac(m, d, jacp, jacr, point, body)
    ccall((:mj_jac, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, point, body)
end
function mj_jacBody(m, d, jacp, jacr, body)
    ccall((:mj_jacBody, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, body)
end
function mj_jacBodyCom(m, d, jacp, jacr, body)
    ccall((:mj_jacBodyCom, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, body)
end
function mj_jacSubtreeCom(m, d, jacp, body)
    ccall((:mj_jacSubtreeCom, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cint), m, d, jacp, body)
end
function mj_jacGeom(m, d, jacp, jacr, geom)
    ccall((:mj_jacGeom, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, geom)
end
function mj_jacSite(m, d, jacp, jacr, site)
    ccall((:mj_jacSite, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, site)
end
function mj_jacPointAxis(m, d, jacPoint, jacAxis, point, axis, body)
    ccall((:mj_jacPointAxis, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacPoint, jacAxis, point, axis, body)
end
function mj_name2id(m, type, name)
    ccall((:mj_name2id, libmujuco), Cint, (Ptr{mjModel}, Cint, Ptr{Cchar}), m, type, name)
end
function mj_id2name(m, type, id)
    ccall((:mj_id2name, libmujuco), Ptr{Cchar}, (Ptr{mjModel}, Cint, Cint), m, type, id)
end
function mj_fullM(m, dst, M)
    ccall((:mj_fullM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}), m, dst, M)
end
function mj_mulM(m, d, res, vec)
    ccall((:mj_mulM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
function mj_mulM2(m, d, res, vec)
    ccall((:mj_mulM2, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
function mj_addM(m, d, dst, rownnz, rowadr, colind)
    ccall((:mj_addM, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), m, d, dst, rownnz, rowadr, colind)
end
function mj_applyFT(m, d, force, torque, point, body, qfrc_target)
    ccall((:mj_applyFT, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}), m, d, force, torque, point, body, qfrc_target)
end
function mj_objectVelocity(m, d, objtype, objid, res, flg_local)
    ccall((:mj_objectVelocity, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint), m, d, objtype, objid, res, flg_local)
end
function mj_objectAcceleration(m, d, objtype, objid, res, flg_local)
    ccall((:mj_objectAcceleration, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint), m, d, objtype, objid, res, flg_local)
end
function mj_contactForce(m, d, id, result)
    ccall((:mj_contactForce, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}), m, d, id, result)
end
function mj_differentiatePos(m, qvel, dt, qpos1, qpos2)
    ccall((:mj_differentiatePos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}), m, qvel, dt, qpos1, qpos2)
end
function mj_integratePos(m, qpos, qvel, dt)
    ccall((:mj_integratePos, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), m, qpos, qvel, dt)
end
function mj_normalizeQuat(m, qpos)
    ccall((:mj_normalizeQuat, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}), m, qpos)
end
function mj_local2Global(d, xpos, xmat, pos, quat, body, sameframe)
    ccall((:mj_local2Global, libmujuco), Cvoid, (Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, mjtByte), d, xpos, xmat, pos, quat, body, sameframe)
end
function mj_getTotalmass(m)
    ccall((:mj_getTotalmass, libmujuco), mjtNum, (Ptr{mjModel},), m)
end
function mj_setTotalmass(m, newmass)
    ccall((:mj_setTotalmass, libmujuco), Cvoid, (Ptr{mjModel}, mjtNum), m, newmass)
end
function mj_getPluginConfig(m, plugin_id, attrib)
    ccall((:mj_getPluginConfig, libmujuco), Ptr{Cchar}, (Ptr{mjModel}, Cint, Ptr{Cchar}), m, plugin_id, attrib)
end
function mj_loadPluginLibrary(path)
    ccall((:mj_loadPluginLibrary, libmujuco), Cvoid, (Ptr{Cchar},), path)
end
function mj_loadAllPluginLibraries(directory, callback)
    ccall((:mj_loadAllPluginLibraries, libmujuco), Cvoid, (Ptr{Cchar}, mjfPluginLibraryLoadCallback), directory, callback)
end
function mj_version()
    ccall((:mj_version, libmujuco), Cint, ())
end
function mj_versionString()
    ccall((:mj_versionString, libmujuco), Ptr{Cchar}, ())
end
function mj_multiRay(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid, dist, nray, cutoff)
    ccall((:mj_multiRay, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtByte}, mjtByte, Cint, Ptr{Cint}, Ptr{mjtNum}, Cint, mjtNum), m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid, dist, nray, cutoff)
end
function mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)
    ccall((:mj_ray, libmujuco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtByte}, mjtByte, Cint, Ptr{Cint}), m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)
end
function mj_rayHfield(m, d, geomid, pnt, vec)
    ccall((:mj_rayHfield, libmujuco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), m, d, geomid, pnt, vec)
end
function mj_rayMesh(m, d, geomid, pnt, vec)
    ccall((:mj_rayMesh, libmujuco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), m, d, geomid, pnt, vec)
end
function mju_rayGeom(pos, mat, size, pnt, vec, geomtype)
    ccall((:mju_rayGeom, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), pos, mat, size, pnt, vec, geomtype)
end
function mju_raySkin(nface, nvert, face, vert, pnt, vec, vertid)
    ccall((:mju_raySkin, libmujuco), mjtNum, (Cint, Cint, Ptr{Cint}, Ptr{Cfloat}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}), nface, nvert, face, vert, pnt, vec, vertid)
end
function mjv_defaultCamera(cam)
    ccall((:mjv_defaultCamera, libmujuco), Cvoid, (Ptr{mjvCamera},), cam)
end
function mjv_defaultFreeCamera(m, cam)
    ccall((:mjv_defaultFreeCamera, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjvCamera}), m, cam)
end
function mjv_defaultPerturb(pert)
    ccall((:mjv_defaultPerturb, libmujuco), Cvoid, (Ptr{mjvPerturb},), pert)
end
function mjv_room2model(modelpos, modelquat, roompos, roomquat, scn)
    ccall((:mjv_room2model, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), modelpos, modelquat, roompos, roomquat, scn)
end
function mjv_model2room(roompos, roomquat, modelpos, modelquat, scn)
    ccall((:mjv_model2room, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), roompos, roomquat, modelpos, modelquat, scn)
end
function mjv_cameraInModel(headpos, forward, up, scn)
    ccall((:mjv_cameraInModel, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), headpos, forward, up, scn)
end
function mjv_cameraInRoom(headpos, forward, up, scn)
    ccall((:mjv_cameraInRoom, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), headpos, forward, up, scn)
end
function mjv_frustumHeight(scn)
    ccall((:mjv_frustumHeight, libmujuco), mjtNum, (Ptr{mjvScene},), scn)
end
function mjv_alignToCamera(res, vec, forward)
    ccall((:mjv_alignToCamera, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, forward)
end
function mjv_moveCamera(m, action, reldx, reldy, scn, cam)
    ccall((:mjv_moveCamera, libmujuco), Cvoid, (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}), m, action, reldx, reldy, scn, cam)
end
function mjv_moveCameraFromState(scnstate, action, reldx, reldy, scn, cam)
    ccall((:mjv_moveCameraFromState, libmujuco), Cvoid, (Ptr{mjvSceneState}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}), scnstate, action, reldx, reldy, scn, cam)
end
function mjv_movePerturb(m, d, action, reldx, reldy, scn, pert)
    ccall((:mjv_movePerturb, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}), m, d, action, reldx, reldy, scn, pert)
end
function mjv_movePerturbFromState(scnstate, action, reldx, reldy, scn, pert)
    ccall((:mjv_movePerturbFromState, libmujuco), Cvoid, (Ptr{mjvSceneState}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}), scnstate, action, reldx, reldy, scn, pert)
end
function mjv_moveModel(m, action, reldx, reldy, roomup, scn)
    ccall((:mjv_moveModel, libmujuco), Cvoid, (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjtNum}, Ptr{mjvScene}), m, action, reldx, reldy, roomup, scn)
end
function mjv_initPerturb(m, d, scn, pert)
    ccall((:mjv_initPerturb, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}, Ptr{mjvPerturb}), m, d, scn, pert)
end
function mjv_applyPerturbPose(m, d, pert, flg_paused)
    ccall((:mjv_applyPerturbPose, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}, Cint), m, d, pert, flg_paused)
end
function mjv_applyPerturbForce(m, d, pert)
    ccall((:mjv_applyPerturbForce, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}), m, d, pert)
end
function mjv_averageCamera(cam1, cam2)
    ccall((:mjv_averageCamera, libmujuco), mjvGLCamera, (Ptr{mjvGLCamera}, Ptr{mjvGLCamera}), cam1, cam2)
end
function mjv_select(m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid)
    ccall((:mjv_select, libmujuco), Cint, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, mjtNum, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}), m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid)
end
function mjv_defaultOption(opt)
    ccall((:mjv_defaultOption, libmujuco), Cvoid, (Ptr{mjvOption},), opt)
end
function mjv_defaultFigure(fig)
    ccall((:mjv_defaultFigure, libmujuco), Cvoid, (Ptr{mjvFigure},), fig)
end
function mjv_initGeom(geom, type, size, pos, mat, rgba)
    ccall((:mjv_initGeom, libmujuco), Cvoid, (Ptr{mjvGeom}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cfloat}), geom, type, size, pos, mat, rgba)
end
function mjv_makeConnector(geom, type, width, a0, a1, a2, b0, b1, b2)
    ccall((:mjv_makeConnector, libmujuco), Cvoid, (Ptr{mjvGeom}, Cint, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum), geom, type, width, a0, a1, a2, b0, b1, b2)
end
function mjv_connector(geom, type, width, from, to)
    ccall((:mjv_connector, libmujuco), Cvoid, (Ptr{mjvGeom}, Cint, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}), geom, type, width, from, to)
end
function mjv_defaultScene(scn)
    ccall((:mjv_defaultScene, libmujuco), Cvoid, (Ptr{mjvScene},), scn)
end
function mjv_makeScene(m, scn, maxgeom)
    ccall((:mjv_makeScene, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjvScene}, Cint), m, scn, maxgeom)
end
function mjv_freeScene(scn)
    ccall((:mjv_freeScene, libmujuco), Cvoid, (Ptr{mjvScene},), scn)
end
function mjv_updateScene(m, d, opt, pert, cam, catmask, scn)
    ccall((:mjv_updateScene, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Ptr{mjvCamera}, Cint, Ptr{mjvScene}), m, d, opt, pert, cam, catmask, scn)
end
function mjv_updateSceneFromState(scnstate, opt, pert, cam, catmask, scn)
    ccall((:mjv_updateSceneFromState, libmujuco), Cint, (Ptr{mjvSceneState}, Ptr{mjvOption}, Ptr{mjvPerturb}, Ptr{mjvCamera}, Cint, Ptr{mjvScene}), scnstate, opt, pert, cam, catmask, scn)
end
function mjv_defaultSceneState(scnstate)
    ccall((:mjv_defaultSceneState, libmujuco), Cvoid, (Ptr{mjvSceneState},), scnstate)
end
function mjv_makeSceneState(m, d, scnstate, maxgeom)
    ccall((:mjv_makeSceneState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvSceneState}, Cint), m, d, scnstate, maxgeom)
end
function mjv_freeSceneState(scnstate)
    ccall((:mjv_freeSceneState, libmujuco), Cvoid, (Ptr{mjvSceneState},), scnstate)
end
function mjv_updateSceneState(m, d, opt, scnstate)
    ccall((:mjv_updateSceneState, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvSceneState}), m, d, opt, scnstate)
end
function mjv_addGeoms(m, d, opt, pert, catmask, scn)
    ccall((:mjv_addGeoms, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Cint, Ptr{mjvScene}), m, d, opt, pert, catmask, scn)
end
function mjv_makeLights(m, d, scn)
    ccall((:mjv_makeLights, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}), m, d, scn)
end
function mjv_updateCamera(m, d, cam, scn)
    ccall((:mjv_updateCamera, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvCamera}, Ptr{mjvScene}), m, d, cam, scn)
end
function mjv_updateSkin(m, d, scn)
    ccall((:mjv_updateSkin, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}), m, d, scn)
end
function mjr_defaultContext(con)
    ccall((:mjr_defaultContext, libmujuco), Cvoid, (Ptr{mjrContext},), con)
end
function mjr_makeContext(m, con, fontscale)
    ccall((:mjr_makeContext, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, fontscale)
end
function mjr_changeFont(fontscale, con)
    ccall((:mjr_changeFont, libmujuco), Cvoid, (Cint, Ptr{mjrContext}), fontscale, con)
end
function mjr_addAux(index, width, height, samples, con)
    ccall((:mjr_addAux, libmujuco), Cvoid, (Cint, Cint, Cint, Cint, Ptr{mjrContext}), index, width, height, samples, con)
end
function mjr_freeContext(con)
    ccall((:mjr_freeContext, libmujuco), Cvoid, (Ptr{mjrContext},), con)
end
function mjr_resizeOffscreen(width, height, con)
    ccall((:mjr_resizeOffscreen, libmujuco), Cvoid, (Cint, Cint, Ptr{mjrContext}), width, height, con)
end
function mjr_uploadTexture(m, con, texid)
    ccall((:mjr_uploadTexture, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, texid)
end
function mjr_uploadMesh(m, con, meshid)
    ccall((:mjr_uploadMesh, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, meshid)
end
function mjr_uploadHField(m, con, hfieldid)
    ccall((:mjr_uploadHField, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, hfieldid)
end
function mjr_restoreBuffer(con)
    ccall((:mjr_restoreBuffer, libmujuco), Cvoid, (Ptr{mjrContext},), con)
end
function mjr_setBuffer(framebuffer, con)
    ccall((:mjr_setBuffer, libmujuco), Cvoid, (Cint, Ptr{mjrContext}), framebuffer, con)
end
function mjr_readPixels(rgb, depth, viewport, con)
    ccall((:mjr_readPixels, libmujuco), Cvoid, (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}), rgb, depth, viewport, con)
end
function mjr_drawPixels(rgb, depth, viewport, con)
    ccall((:mjr_drawPixels, libmujuco), Cvoid, (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}), rgb, depth, viewport, con)
end
function mjr_blitBuffer(src, dst, flg_color, flg_depth, con)
    ccall((:mjr_blitBuffer, libmujuco), Cvoid, (mjrRect, mjrRect, Cint, Cint, Ptr{mjrContext}), src, dst, flg_color, flg_depth, con)
end
function mjr_setAux(index, con)
    ccall((:mjr_setAux, libmujuco), Cvoid, (Cint, Ptr{mjrContext}), index, con)
end
function mjr_blitAux(index, src, left, bottom, con)
    ccall((:mjr_blitAux, libmujuco), Cvoid, (Cint, mjrRect, Cint, Cint, Ptr{mjrContext}), index, src, left, bottom, con)
end
function mjr_text(font, txt, con, x, y, r, g, b)
    ccall((:mjr_text, libmujuco), Cvoid, (Cint, Ptr{Cchar}, Ptr{mjrContext}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat), font, txt, con, x, y, r, g, b)
end
function mjr_overlay(font, gridpos, viewport, overlay, overlay2, con)
    ccall((:mjr_overlay, libmujuco), Cvoid, (Cint, Cint, mjrRect, Ptr{Cchar}, Ptr{Cchar}, Ptr{mjrContext}), font, gridpos, viewport, overlay, overlay2, con)
end
function mjr_maxViewport(con)
    ccall((:mjr_maxViewport, libmujuco), mjrRect, (Ptr{mjrContext},), con)
end
function mjr_rectangle(viewport, r, g, b, a)
    ccall((:mjr_rectangle, libmujuco), Cvoid, (mjrRect, Cfloat, Cfloat, Cfloat, Cfloat), viewport, r, g, b, a)
end
function mjr_label(viewport, font, txt, r, g, b, a, rt, gt, bt, con)
    ccall((:mjr_label, libmujuco), Cvoid, (mjrRect, Cint, Ptr{Cchar}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Ptr{mjrContext}), viewport, font, txt, r, g, b, a, rt, gt, bt, con)
end
function mjr_figure(viewport, fig, con)
    ccall((:mjr_figure, libmujuco), Cvoid, (mjrRect, Ptr{mjvFigure}, Ptr{mjrContext}), viewport, fig, con)
end
function mjr_render(viewport, scn, con)
    ccall((:mjr_render, libmujuco), Cvoid, (mjrRect, Ptr{mjvScene}, Ptr{mjrContext}), viewport, scn, con)
end
function mjr_finish()
    ccall((:mjr_finish, libmujuco), Cvoid, ())
end
function mjr_getError()
    ccall((:mjr_getError, libmujuco), Cint, ())
end
function mjr_findRect(x, y, nrect, rect)
    ccall((:mjr_findRect, libmujuco), Cint, (Cint, Cint, Cint, Ptr{mjrRect}), x, y, nrect, rect)
end
function mjui_themeSpacing(ind)
    ccall((:mjui_themeSpacing, libmujuco), mjuiThemeSpacing, (Cint,), ind)
end
function mjui_themeColor(ind)
    ccall((:mjui_themeColor, libmujuco), mjuiThemeColor, (Cint,), ind)
end
function mjui_add(ui, def)
    ccall((:mjui_add, libmujuco), Cvoid, (Ptr{mjUI}, Ptr{mjuiDef}), ui, def)
end
function mjui_addToSection(ui, sect, def)
    ccall((:mjui_addToSection, libmujuco), Cvoid, (Ptr{mjUI}, Cint, Ptr{mjuiDef}), ui, sect, def)
end
function mjui_resize(ui, con)
    ccall((:mjui_resize, libmujuco), Cvoid, (Ptr{mjUI}, Ptr{mjrContext}), ui, con)
end
function mjui_update(section, item, ui, state, con)
    ccall((:mjui_update, libmujuco), Cvoid, (Cint, Cint, Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), section, item, ui, state, con)
end
function mjui_event(ui, state, con)
    ccall((:mjui_event, libmujuco), Ptr{mjuiItem}, (Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), ui, state, con)
end
function mjui_render(ui, state, con)
    ccall((:mjui_render, libmujuco), Cvoid, (Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), ui, state, con)
end
function mju_error_i(msg, i)
    ccall((:mju_error_i, libmujuco), Cvoid, (Ptr{Cchar}, Cint), msg, i)
end
function mju_error_s(msg, text)
    ccall((:mju_error_s, libmujuco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), msg, text)
end
function mju_warning_i(msg, i)
    ccall((:mju_warning_i, libmujuco), Cvoid, (Ptr{Cchar}, Cint), msg, i)
end
function mju_warning_s(msg, text)
    ccall((:mju_warning_s, libmujuco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), msg, text)
end
function mju_clearHandlers()
    ccall((:mju_clearHandlers, libmujuco), Cvoid, ())
end
function mju_malloc(size)
    ccall((:mju_malloc, libmujuco), Ptr{Cvoid}, (Csize_t,), size)
end
function mju_free(ptr)
    ccall((:mju_free, libmujuco), Cvoid, (Ptr{Cvoid},), ptr)
end
function mj_warning(d, warning, info)
    ccall((:mj_warning, libmujuco), Cvoid, (Ptr{mjData}, Cint, Cint), d, warning, info)
end
function mju_writeLog(type, msg)
    ccall((:mju_writeLog, libmujuco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), type, msg)
end
function mju_zero3(res)
    ccall((:mju_zero3, libmujuco), Cvoid, (Ptr{mjtNum},), res)
end
function mju_copy3(res, data)
    ccall((:mju_copy3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, data)
end
function mju_scl3(res, vec, scl)
    ccall((:mju_scl3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec, scl)
end
function mju_add3(res, vec1, vec2)
    ccall((:mju_add3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec1, vec2)
end
function mju_sub3(res, vec1, vec2)
    ccall((:mju_sub3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec1, vec2)
end
function mju_addTo3(res, vec)
    ccall((:mju_addTo3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, vec)
end
function mju_subFrom3(res, vec)
    ccall((:mju_subFrom3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, vec)
end
function mju_addToScl3(res, vec, scl)
    ccall((:mju_addToScl3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec, scl)
end
function mju_addScl3(res, vec1, vec2, scl)
    ccall((:mju_addScl3, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec1, vec2, scl)
end
function mju_normalize3(res)
    ccall((:mju_normalize3, libmujuco), mjtNum, (Ptr{mjtNum},), res)
end
function mju_norm3(vec)
    ccall((:mju_norm3, libmujuco), mjtNum, (Ptr{mjtNum},), vec)
end
function mju_dot3(vec1, vec2)
    ccall((:mju_dot3, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}), vec1, vec2)
end
function mju_dist3(pos1, pos2)
    ccall((:mju_dist3, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}), pos1, pos2)
end
function mju_rotVecMat(res, vec, mat)
    ccall((:mju_rotVecMat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, mat)
end
function mju_rotVecMatT(res, vec, mat)
    ccall((:mju_rotVecMatT, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, mat)
end
function mju_cross(res, a, b)
    ccall((:mju_cross, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, a, b)
end
function mju_zero4(res)
    ccall((:mju_zero4, libmujuco), Cvoid, (Ptr{mjtNum},), res)
end
function mju_unit4(res)
    ccall((:mju_unit4, libmujuco), Cvoid, (Ptr{mjtNum},), res)
end
function mju_copy4(res, data)
    ccall((:mju_copy4, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, data)
end
function mju_normalize4(res)
    ccall((:mju_normalize4, libmujuco), mjtNum, (Ptr{mjtNum},), res)
end
function mju_zero(res, n)
    ccall((:mju_zero, libmujuco), Cvoid, (Ptr{mjtNum}, Cint), res, n)
end
function mju_fill(res, val, n)
    ccall((:mju_fill, libmujuco), Cvoid, (Ptr{mjtNum}, mjtNum, Cint), res, val, n)
end
function mju_copy(res, data, n)
    ccall((:mju_copy, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, data, n)
end
function mju_sum(vec, n)
    ccall((:mju_sum, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), vec, n)
end
function mju_L1(vec, n)
    ccall((:mju_L1, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), vec, n)
end
function mju_scl(res, vec, scl, n)
    ccall((:mju_scl, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec, scl, n)
end
function mju_add(res, vec1, vec2, n)
    ccall((:mju_add, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec1, vec2, n)
end
function mju_sub(res, vec1, vec2, n)
    ccall((:mju_sub, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec1, vec2, n)
end
function mju_addTo(res, vec, n)
    ccall((:mju_addTo, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec, n)
end
function mju_subFrom(res, vec, n)
    ccall((:mju_subFrom, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec, n)
end
function mju_addToScl(res, vec, scl, n)
    ccall((:mju_addToScl, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec, scl, n)
end
function mju_addScl(res, vec1, vec2, scl, n)
    ccall((:mju_addScl, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec1, vec2, scl, n)
end
function mju_normalize(res, n)
    ccall((:mju_normalize, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), res, n)
end
function mju_norm(res, n)
    ccall((:mju_norm, libmujuco), mjtNum, (Ptr{mjtNum}, Cint), res, n)
end
function mju_dot(vec1, vec2, n)
    ccall((:mju_dot, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), vec1, vec2, n)
end
function mju_mulMatVec(res, mat, vec, nr, nc)
    ccall((:mju_mulMatVec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, vec, nr, nc)
end
function mju_mulMatTVec(res, mat, vec, nr, nc)
    ccall((:mju_mulMatTVec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, vec, nr, nc)
end
function mju_mulVecMatVec(vec1, mat, vec2, n)
    ccall((:mju_mulVecMatVec, libmujuco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), vec1, mat, vec2, n)
end
function mju_transpose(res, mat, nr, nc)
    ccall((:mju_transpose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, nr, nc)
end
function mju_symmetrize(res, mat, n)
    ccall((:mju_symmetrize, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, mat, n)
end
function mju_eye(mat, n)
    ccall((:mju_eye, libmujuco), Cvoid, (Ptr{mjtNum}, Cint), mat, n)
end
function mju_mulMatMat(res, mat1, mat2, r1, c1, c2)
    ccall((:mju_mulMatMat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, c2)
end
function mju_mulMatMatT(res, mat1, mat2, r1, c1, r2)
    ccall((:mju_mulMatMatT, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, r2)
end
function mju_mulMatTMat(res, mat1, mat2, r1, c1, c2)
    ccall((:mju_mulMatTMat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, c2)
end
function mju_sqrMatTD(res, mat, diag, nr, nc)
    ccall((:mju_sqrMatTD, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, diag, nr, nc)
end
function mju_transformSpatial(res, vec, flg_force, newpos, oldpos, rotnew2old)
    ccall((:mju_transformSpatial, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, flg_force, newpos, oldpos, rotnew2old)
end
function mju_rotVecQuat(res, vec, quat)
    ccall((:mju_rotVecQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, quat)
end
function mju_negQuat(res, quat)
    ccall((:mju_negQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, quat)
end
function mju_mulQuat(res, quat1, quat2)
    ccall((:mju_mulQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat1, quat2)
end
function mju_mulQuatAxis(res, quat, axis)
    ccall((:mju_mulQuatAxis, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat, axis)
end
function mju_axisAngle2Quat(res, axis, angle)
    ccall((:mju_axisAngle2Quat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, axis, angle)
end
function mju_quat2Vel(res, quat, dt)
    ccall((:mju_quat2Vel, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, quat, dt)
end
function mju_subQuat(res, qa, qb)
    ccall((:mju_subQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, qa, qb)
end
function mju_quat2Mat(res, quat)
    ccall((:mju_quat2Mat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, quat)
end
function mju_mat2Quat(quat, mat)
    ccall((:mju_mat2Quat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), quat, mat)
end
function mju_derivQuat(res, quat, vel)
    ccall((:mju_derivQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat, vel)
end
function mju_quatIntegrate(quat, vel, scale)
    ccall((:mju_quatIntegrate, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), quat, vel, scale)
end
function mju_quatZ2Vec(quat, vec)
    ccall((:mju_quatZ2Vec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), quat, vec)
end
function mju_mulPose(posres, quatres, pos1, quat1, pos2, quat2)
    ccall((:mju_mulPose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), posres, quatres, pos1, quat1, pos2, quat2)
end
function mju_negPose(posres, quatres, pos, quat)
    ccall((:mju_negPose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), posres, quatres, pos, quat)
end
function mju_trnVecPose(res, pos, quat, vec)
    ccall((:mju_trnVecPose, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, pos, quat, vec)
end
function mju_cholFactor(mat, n, mindiag)
    ccall((:mju_cholFactor, libmujuco), Cint, (Ptr{mjtNum}, Cint, mjtNum), mat, n, mindiag)
end
function mju_cholSolve(res, mat, vec, n)
    ccall((:mju_cholSolve, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, mat, vec, n)
end
function mju_cholUpdate(mat, x, n, flg_plus)
    ccall((:mju_cholUpdate, libmujuco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), mat, x, n, flg_plus)
end
function mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)
    ccall((:mju_cholFactorBand, libmujuco), mjtNum, (Ptr{mjtNum}, Cint, Cint, Cint, mjtNum, mjtNum), mat, ntotal, nband, ndense, diagadd, diagmul)
end
function mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)
    ccall((:mju_cholSolveBand, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat, vec, ntotal, nband, ndense)
end
function mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)
    ccall((:mju_band2Dense, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint, mjtByte), res, mat, ntotal, nband, ndense, flg_sym)
end
function mju_dense2Band(res, mat, ntotal, nband, ndense)
    ccall((:mju_dense2Band, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat, ntotal, nband, ndense)
end
function mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nvec, flg_sym)
    ccall((:mju_bandMulMatVec, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint, Cint, mjtByte), res, mat, vec, ntotal, nband, ndense, nvec, flg_sym)
end
function mju_bandDiag(i, ntotal, nband, ndense)
    ccall((:mju_bandDiag, libmujuco), Cint, (Cint, Cint, Cint, Cint), i, ntotal, nband, ndense)
end
function mju_eig3(eigval, eigvec, quat, mat)
    ccall((:mju_eig3, libmujuco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), eigval, eigvec, quat, mat)
end
function mju_boxQP(res, R, index, H, g, n, lower, upper)
    ccall((:mju_boxQP, libmujuco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), res, R, index, H, g, n, lower, upper)
end
function mju_boxQPmalloc(res, R, index, H, g, n, lower, upper)
    ccall((:mju_boxQPmalloc, libmujuco), Cvoid, (Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}, Ptr{Ptr{Cint}}, Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}, Cint, Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}), res, R, index, H, g, n, lower, upper)
end
function mju_muscleGain(len, vel, lengthrange, acc0, prm)
    ccall((:mju_muscleGain, libmujuco), mjtNum, (mjtNum, mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}), len, vel, lengthrange, acc0, prm)
end
function mju_muscleBias(len, lengthrange, acc0, prm)
    ccall((:mju_muscleBias, libmujuco), mjtNum, (mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}), len, lengthrange, acc0, prm)
end
function mju_muscleDynamics(ctrl, act, prm)
    ccall((:mju_muscleDynamics, libmujuco), mjtNum, (mjtNum, mjtNum, Ptr{mjtNum}), ctrl, act, prm)
end
function mju_encodePyramid(pyramid, force, mu, dim)
    ccall((:mju_encodePyramid, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), pyramid, force, mu, dim)
end
function mju_decodePyramid(force, pyramid, mu, dim)
    ccall((:mju_decodePyramid, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), force, pyramid, mu, dim)
end
function mju_springDamper(pos0, vel0, Kp, Kv, dt)
    ccall((:mju_springDamper, libmujuco), mjtNum, (mjtNum, mjtNum, mjtNum, mjtNum, mjtNum), pos0, vel0, Kp, Kv, dt)
end
function mju_min(a, b)
    ccall((:mju_min, libmujuco), mjtNum, (mjtNum, mjtNum), a, b)
end
function mju_max(a, b)
    ccall((:mju_max, libmujuco), mjtNum, (mjtNum, mjtNum), a, b)
end
function mju_clip(x, min, max)
    ccall((:mju_clip, libmujuco), mjtNum, (mjtNum, mjtNum, mjtNum), x, min, max)
end
function mju_sign(x)
    ccall((:mju_sign, libmujuco), mjtNum, (mjtNum,), x)
end
function mju_round(x)
    ccall((:mju_round, libmujuco), Cint, (mjtNum,), x)
end
function mju_type2Str(type)
    ccall((:mju_type2Str, libmujuco), Ptr{Cchar}, (Cint,), type)
end
function mju_str2Type(str)
    ccall((:mju_str2Type, libmujuco), Cint, (Ptr{Cchar},), str)
end
function mju_writeNumBytes(nbytes)
    ccall((:mju_writeNumBytes, libmujuco), Ptr{Cchar}, (Csize_t,), nbytes)
end
function mju_warningText(warning, info)
    ccall((:mju_warningText, libmujuco), Ptr{Cchar}, (Cint, Csize_t), warning, info)
end
function mju_isBad(x)
    ccall((:mju_isBad, libmujuco), Cint, (mjtNum,), x)
end
function mju_isZero(vec, n)
    ccall((:mju_isZero, libmujuco), Cint, (Ptr{mjtNum}, Cint), vec, n)
end
function mju_standardNormal(num2)
    ccall((:mju_standardNormal, libmujuco), mjtNum, (Ptr{mjtNum},), num2)
end
function mju_f2n(res, vec, n)
    ccall((:mju_f2n, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{Cfloat}, Cint), res, vec, n)
end
function mju_n2f(res, vec, n)
    ccall((:mju_n2f, libmujuco), Cvoid, (Ptr{Cfloat}, Ptr{mjtNum}, Cint), res, vec, n)
end
function mju_d2n(res, vec, n)
    ccall((:mju_d2n, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{Cdouble}, Cint), res, vec, n)
end
function mju_n2d(res, vec, n)
    ccall((:mju_n2d, libmujuco), Cvoid, (Ptr{Cdouble}, Ptr{mjtNum}, Cint), res, vec, n)
end
function mju_insertionSort(list, n)
    ccall((:mju_insertionSort, libmujuco), Cvoid, (Ptr{mjtNum}, Cint), list, n)
end
function mju_insertionSortInt(list, n)
    ccall((:mju_insertionSortInt, libmujuco), Cvoid, (Ptr{Cint}, Cint), list, n)
end
function mju_Halton(index, base)
    ccall((:mju_Halton, libmujuco), mjtNum, (Cint, Cint), index, base)
end
function mju_strncpy(dst, src, n)
    ccall((:mju_strncpy, libmujuco), Ptr{Cchar}, (Ptr{Cchar}, Ptr{Cchar}, Cint), dst, src, n)
end
function mju_sigmoid(x)
    ccall((:mju_sigmoid, libmujuco), mjtNum, (mjtNum,), x)
end
function mjd_transitionFD(m, d, eps, flg_centered, A, B, C, D)
    ccall((:mjd_transitionFD, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, mjtNum, mjtByte, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, eps, flg_centered, A, B, C, D)
end
function mjd_inverseFD(m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)
    ccall((:mjd_inverseFD, libmujuco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, mjtNum, mjtByte, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)
end
function mjd_subQuat(qa, qb, Da, Db)
    ccall((:mjd_subQuat, libmujuco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), qa, qb, Da, Db)
end
function mjd_quatIntegrate(vel, scale, Dquat, Dvel, Dscale)
    ccall((:mjd_quatIntegrate, libmujuco), Cvoid, (Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), vel, scale, Dquat, Dvel, Dscale)
end
function mjp_defaultPlugin(plugin)
    ccall((:mjp_defaultPlugin, libmujuco), Cvoid, (Ptr{mjpPlugin},), plugin)
end
function mjp_registerPlugin(plugin)
    ccall((:mjp_registerPlugin, libmujuco), Cint, (Ptr{mjpPlugin},), plugin)
end
function mjp_pluginCount()
    ccall((:mjp_pluginCount, libmujuco), Cint, ())
end
function mjp_getPlugin(name, slot)
    ccall((:mjp_getPlugin, libmujuco), Ptr{mjpPlugin}, (Ptr{Cchar}, Ptr{Cint}), name, slot)
end
function mjp_getPluginAtSlot(slot)
    ccall((:mjp_getPluginAtSlot, libmujuco), Ptr{mjpPlugin}, (Cint,), slot)
end
function mjp_defaultResourceProvider(provider)
    ccall((:mjp_defaultResourceProvider, libmujuco), Cvoid, (Ptr{mjpResourceProvider},), provider)
end
function mjp_registerResourceProvider(provider)
    ccall((:mjp_registerResourceProvider, libmujuco), Cint, (Ptr{mjpResourceProvider},), provider)
end
function mjp_resourceProviderCount()
    ccall((:mjp_resourceProviderCount, libmujuco), Cint, ())
end
function mjp_getResourceProvider(resource_name)
    ccall((:mjp_getResourceProvider, libmujuco), Ptr{mjpResourceProvider}, (Ptr{Cchar},), resource_name)
end
function mjp_getResourceProviderAtSlot(slot)
    ccall((:mjp_getResourceProviderAtSlot, libmujuco), Ptr{mjpResourceProvider}, (Cint,), slot)
end
