function Base.getproperty(x::Ptr{var"##Ctag#247"}, f::Symbol)
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
function Base.getproperty(x::var"##Ctag#247", f::Symbol)
    r = Ref{var"##Ctag#247"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#247"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#247"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#248"}, f::Symbol)
    f === :shadowsize && return Ptr{Cint}(x + 0)
    f === :offsamples && return Ptr{Cint}(x + 4)
    f === :numslices && return Ptr{Cint}(x + 8)
    f === :numstacks && return Ptr{Cint}(x + 12)
    f === :numquads && return Ptr{Cint}(x + 16)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#248", f::Symbol)
    r = Ref{var"##Ctag#248"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#248"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#248"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#249"}, f::Symbol)
    f === :ambient && return Ptr{NTuple{3, Cfloat}}(x + 0)
    f === :diffuse && return Ptr{NTuple{3, Cfloat}}(x + 12)
    f === :specular && return Ptr{NTuple{3, Cfloat}}(x + 24)
    f === :active && return Ptr{Cint}(x + 36)
    return getfield(x, f)
end
function Base.getproperty(x::var"##Ctag#249", f::Symbol)
    r = Ref{var"##Ctag#249"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#249"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#249"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#250"}, f::Symbol)
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
function Base.getproperty(x::var"##Ctag#250", f::Symbol)
    r = Ref{var"##Ctag#250"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#250"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#250"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#251"}, f::Symbol)
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
function Base.getproperty(x::var"##Ctag#251", f::Symbol)
    r = Ref{var"##Ctag#251"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#251"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#251"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#252"}, f::Symbol)
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
function Base.getproperty(x::var"##Ctag#252", f::Symbol)
    r = Ref{var"##Ctag#252"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#252"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#252"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{mjVisual_}, f::Symbol)
    f === :_global && return Ptr{var"##Ctag#247"}(x + 0)
    f === :quality && return Ptr{var"##Ctag#248"}(x + 40)
    f === :headlight && return Ptr{var"##Ctag#249"}(x + 60)
    f === :map && return Ptr{var"##Ctag#250"}(x + 100)
    f === :scale && return Ptr{var"##Ctag#251"}(x + 152)
    f === :rgba && return Ptr{var"##Ctag#252"}(x + 216)
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
function Base.getproperty(x::Ptr{var"##Ctag#253"}, f::Symbol)
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
function Base.getproperty(x::var"##Ctag#253", f::Symbol)
    r = Ref{var"##Ctag#253"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#253"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#253"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{var"##Ctag#254"}, f::Symbol)
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
function Base.getproperty(x::var"##Ctag#254", f::Symbol)
    r = Ref{var"##Ctag#254"}(x)
    ptr = Base.unsafe_convert(Ptr{var"##Ctag#254"}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end
function Base.setproperty!(x::Ptr{var"##Ctag#254"}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end
function Base.getproperty(x::Ptr{mjvSceneState_}, f::Symbol)
    f === :nbuffer && return Ptr{Cint}(x + 0)
    f === :buffer && return Ptr{Ptr{Cvoid}}(x + 8)
    f === :maxgeom && return Ptr{Cint}(x + 16)
    f === :plugincache && return Ptr{mjvScene}(x + 24)
    f === :model && return Ptr{var"##Ctag#253"}(x + 8680)
    f === :data && return Ptr{var"##Ctag#254"}(x + 10560)
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
"""
Initialize VFS to empty (no deallocation).
"""
function mj_defaultVFS(vfs)
    ccall((:mj_defaultVFS, libmujoco), Cvoid, (Ptr{mjVFS},), vfs)
end
"""
Add file to VFS, return 0: success, 1: full, 2: repeated name, -1: failed to load.
"""
function mj_addFileVFS(vfs, directory, filename)
    ccall((:mj_addFileVFS, libmujoco), Cint, (Ptr{mjVFS}, Ptr{Cchar}, Ptr{Cchar}), vfs, directory, filename)
end
"""
Make empty file in VFS, return 0: success, 1: full, 2: repeated name.
"""
function mj_makeEmptyFileVFS(vfs, filename, filesize)
    ccall((:mj_makeEmptyFileVFS, libmujoco), Cint, (Ptr{mjVFS}, Ptr{Cchar}, Cint), vfs, filename, filesize)
end
"""
Return file index in VFS, or -1 if not found in VFS.
"""
function mj_findFileVFS(vfs, filename)
    ccall((:mj_findFileVFS, libmujoco), Cint, (Ptr{mjVFS}, Ptr{Cchar}), vfs, filename)
end
"""
Delete file from VFS, return 0: success, -1: not found in VFS.
"""
function mj_deleteFileVFS(vfs, filename)
    ccall((:mj_deleteFileVFS, libmujoco), Cint, (Ptr{mjVFS}, Ptr{Cchar}), vfs, filename)
end
"""
Delete all files from VFS.
"""
function mj_deleteVFS(vfs)
    ccall((:mj_deleteVFS, libmujoco), Cvoid, (Ptr{mjVFS},), vfs)
end
"""
Parse XML file in MJCF or URDF format, compile it, return low-level model. If vfs is not NULL, look up files in vfs before reading from disk. If error is not NULL, it must have size error_sz.
"""
function mj_loadXML(filename, vfs, error, error_sz)
    ccall((:mj_loadXML, libmujoco), Ptr{mjModel}, (Ptr{Cchar}, Ptr{mjVFS}, Ptr{Cchar}, Cint), filename, vfs, error, error_sz)
end
"""
Update XML data structures with info from low-level model, save as MJCF. If error is not NULL, it must have size error_sz.
"""
function mj_saveLastXML(filename, m, error, error_sz)
    ccall((:mj_saveLastXML, libmujoco), Cint, (Ptr{Cchar}, Ptr{mjModel}, Ptr{Cchar}, Cint), filename, m, error, error_sz)
end
"""
Free last XML model if loaded. Called internally at each load.
"""
function mj_freeLastXML()
    ccall((:mj_freeLastXML, libmujoco), Cvoid, ())
end
"""
Print internal XML schema as plain text or HTML, with style-padding or &nbsp;.
"""
function mj_printSchema(filename, buffer, buffer_sz, flg_html, flg_pad)
    ccall((:mj_printSchema, libmujoco), Cint, (Ptr{Cchar}, Ptr{Cchar}, Cint, Cint, Cint), filename, buffer, buffer_sz, flg_html, flg_pad)
end
"""
Advance simulation, use control callback to obtain external force and control.
"""
function mj_step(m, d)
    ccall((:mj_step, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Advance simulation in two steps: before external force and control is set by user.
"""
function mj_step1(m, d)
    ccall((:mj_step1, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Advance simulation in two steps: after external force and control is set by user.
"""
function mj_step2(m, d)
    ccall((:mj_step2, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Forward dynamics: same as mj_step but do not integrate in time.
"""
function mj_forward(m, d)
    ccall((:mj_forward, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Inverse dynamics: qacc must be set before calling.
"""
function mj_inverse(m, d)
    ccall((:mj_inverse, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Forward dynamics with skip; skipstage is mjtStage.
"""
function mj_forwardSkip(m, d, skipstage, skipsensor)
    ccall((:mj_forwardSkip, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint), m, d, skipstage, skipsensor)
end
"""
Inverse dynamics with skip; skipstage is mjtStage.
"""
function mj_inverseSkip(m, d, skipstage, skipsensor)
    ccall((:mj_inverseSkip, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint), m, d, skipstage, skipsensor)
end
"""
Set default options for length range computation.
"""
function mj_defaultLROpt(opt)
    ccall((:mj_defaultLROpt, libmujoco), Cvoid, (Ptr{mjLROpt},), opt)
end
"""
Set solver parameters to default values.
"""
function mj_defaultSolRefImp(solref, solimp)
    ccall((:mj_defaultSolRefImp, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), solref, solimp)
end
"""
Set physics options to default values.
"""
function mj_defaultOption(opt)
    ccall((:mj_defaultOption, libmujoco), Cvoid, (Ptr{mjOption},), opt)
end
"""
Set visual options to default values.
"""
function mj_defaultVisual(vis)
    ccall((:mj_defaultVisual, libmujoco), Cvoid, (Ptr{mjVisual},), vis)
end
"""
Copy mjModel, allocate new if dest is NULL.
"""
function mj_copyModel(dest, src)
    ccall((:mj_copyModel, libmujoco), Ptr{mjModel}, (Ptr{mjModel}, Ptr{mjModel}), dest, src)
end
"""
Save model to binary MJB file or memory buffer; buffer has precedence when given.
"""
function mj_saveModel(m, filename, buffer, buffer_sz)
    ccall((:mj_saveModel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}, Ptr{Cvoid}, Cint), m, filename, buffer, buffer_sz)
end
"""
Load model from binary MJB file. If vfs is not NULL, look up file in vfs before reading from disk.
"""
function mj_loadModel(filename, vfs)
    ccall((:mj_loadModel, libmujoco), Ptr{mjModel}, (Ptr{Cchar}, Ptr{mjVFS}), filename, vfs)
end
"""
Free memory allocation in model.
"""
function mj_deleteModel(m)
    ccall((:mj_deleteModel, libmujoco), Cvoid, (Ptr{mjModel},), m)
end
"""
Return size of buffer needed to hold model.
"""
function mj_sizeModel(m)
    ccall((:mj_sizeModel, libmujoco), Cint, (Ptr{mjModel},), m)
end
"""
Allocate mjData corresponding to given model. If the model buffer is unallocated the initial configuration will not be set.
"""
function mj_makeData(m)
    ccall((:mj_makeData, libmujoco), Ptr{mjData}, (Ptr{mjModel},), m)
end
"""
Copy mjData. m is only required to contain the size fields from MJMODEL_INTS.
"""
function mj_copyData(dest, m, src)
    ccall((:mj_copyData, libmujoco), Ptr{mjData}, (Ptr{mjData}, Ptr{mjModel}, Ptr{mjData}), dest, m, src)
end
"""
Reset data to defaults.
"""
function mj_resetData(m, d)
    ccall((:mj_resetData, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Reset data to defaults, fill everything else with debug_value.
"""
function mj_resetDataDebug(m, d, debug_value)
    ccall((:mj_resetDataDebug, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cuchar), m, d, debug_value)
end
"""
Reset data, set fields from specified keyframe.
"""
function mj_resetDataKeyframe(m, d, key)
    ccall((:mj_resetDataKeyframe, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, key)
end
"""
Allocate array of mjtNums on mjData stack. Call mju_error on stack overflow.
"""
function mj_stackAlloc(d, size)
    ccall((:mj_stackAlloc, libmujoco), Ptr{mjtNum}, (Ptr{mjData}, Cint), d, size)
end
"""
Allocate array of ints on mjData stack. Call mju_error on stack overflow.
"""
function mj_stackAllocInt(d, size)
    ccall((:mj_stackAllocInt, libmujoco), Ptr{Cint}, (Ptr{mjData}, Cint), d, size)
end
"""
Free memory allocation in mjData.
"""
function mj_deleteData(d)
    ccall((:mj_deleteData, libmujoco), Cvoid, (Ptr{mjData},), d)
end
"""
Reset all callbacks to NULL pointers (NULL is the default).
"""
function mj_resetCallbacks()
    ccall((:mj_resetCallbacks, libmujoco), Cvoid, ())
end
"""
Set constant fields of mjModel, corresponding to qpos0 configuration.
"""
function mj_setConst(m, d)
    ccall((:mj_setConst, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.
"""
function mj_setLengthRange(m, d, index, opt, error, error_sz)
    ccall((:mj_setLengthRange, libmujoco), Cint, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjLROpt}, Ptr{Cchar}, Cint), m, d, index, opt, error, error_sz)
end
"""
Print mjModel to text file, specifying format. float_format must be a valid printf-style format string for a single float value.
"""
function mj_printFormattedModel(m, filename, float_format)
    ccall((:mj_printFormattedModel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}, Ptr{Cchar}), m, filename, float_format)
end
"""
Print model to text file.
"""
function mj_printModel(m, filename)
    ccall((:mj_printModel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{Cchar}), m, filename)
end
"""
Print mjData to text file, specifying format. float_format must be a valid printf-style format string for a single float value
"""
function mj_printFormattedData(m, d, filename, float_format)
    ccall((:mj_printFormattedData, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{Cchar}, Ptr{Cchar}), m, d, filename, float_format)
end
"""
Print data to text file.
"""
function mj_printData(m, d, filename)
    ccall((:mj_printData, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{Cchar}), m, d, filename)
end
"""
Print matrix to screen.
"""
function mju_printMat(mat, nr, nc)
    ccall((:mju_printMat, libmujoco), Cvoid, (Ptr{mjtNum}, Cint, Cint), mat, nr, nc)
end
"""
Print sparse matrix to screen.
"""
function mju_printMatSparse(mat, nr, rownnz, rowadr, colind)
    ccall((:mju_printMatSparse, libmujoco), Cvoid, (Ptr{mjtNum}, Cint, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), mat, nr, rownnz, rowadr, colind)
end
"""
Run position-dependent computations.
"""
function mj_fwdPosition(m, d)
    ccall((:mj_fwdPosition, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Run velocity-dependent computations.
"""
function mj_fwdVelocity(m, d)
    ccall((:mj_fwdVelocity, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute actuator force qfrc_actuator.
"""
function mj_fwdActuation(m, d)
    ccall((:mj_fwdActuation, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Add up all non-constraint forces, compute qacc_smooth.
"""
function mj_fwdAcceleration(m, d)
    ccall((:mj_fwdAcceleration, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Run selected constraint solver.
"""
function mj_fwdConstraint(m, d)
    ccall((:mj_fwdConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Euler integrator, semi-implicit in velocity.
"""
function mj_Euler(m, d)
    ccall((:mj_Euler, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Runge-Kutta explicit order-N integrator.
"""
function mj_RungeKutta(m, d, N)
    ccall((:mj_RungeKutta, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint), m, d, N)
end
"""
Run position-dependent computations in inverse dynamics.
"""
function mj_invPosition(m, d)
    ccall((:mj_invPosition, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Run velocity-dependent computations in inverse dynamics.
"""
function mj_invVelocity(m, d)
    ccall((:mj_invVelocity, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Apply the analytical formula for inverse constraint dynamics.
"""
function mj_invConstraint(m, d)
    ccall((:mj_invConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compare forward and inverse dynamics, save results in fwdinv.
"""
function mj_compareFwdInv(m, d)
    ccall((:mj_compareFwdInv, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Evaluate position-dependent sensors.
"""
function mj_sensorPos(m, d)
    ccall((:mj_sensorPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Evaluate velocity-dependent sensors.
"""
function mj_sensorVel(m, d)
    ccall((:mj_sensorVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Evaluate acceleration and force-dependent sensors.
"""
function mj_sensorAcc(m, d)
    ccall((:mj_sensorAcc, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Evaluate position-dependent energy (potential).
"""
function mj_energyPos(m, d)
    ccall((:mj_energyPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Evaluate velocity-dependent energy (kinetic).
"""
function mj_energyVel(m, d)
    ccall((:mj_energyVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Check qpos, reset if any element is too big or nan.
"""
function mj_checkPos(m, d)
    ccall((:mj_checkPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Check qvel, reset if any element is too big or nan.
"""
function mj_checkVel(m, d)
    ccall((:mj_checkVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Check qacc, reset if any element is too big or nan.
"""
function mj_checkAcc(m, d)
    ccall((:mj_checkAcc, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Run forward kinematics.
"""
function mj_kinematics(m, d)
    ccall((:mj_kinematics, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Map inertias and motion dofs to global frame centered at CoM.
"""
function mj_comPos(m, d)
    ccall((:mj_comPos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute camera and light positions and orientations.
"""
function mj_camlight(m, d)
    ccall((:mj_camlight, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute tendon lengths, velocities and moment arms.
"""
function mj_tendon(m, d)
    ccall((:mj_tendon, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute actuator transmission lengths and moments.
"""
function mj_transmission(m, d)
    ccall((:mj_transmission, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Run composite rigid body inertia algorithm (CRB).
"""
function mj_crb(m, d)
    ccall((:mj_crb, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute sparse L'*D*L factorizaton of inertia matrix.
"""
function mj_factorM(m, d)
    ccall((:mj_factorM, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y
"""
function mj_solveM(m, d, x, y, n)
    ccall((:mj_solveM, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, x, y, n)
end
"""
Half of linear solve:  x = sqrt(inv(D))*inv(L')*y
"""
function mj_solveM2(m, d, x, y, n)
    ccall((:mj_solveM2, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, x, y, n)
end
"""
Compute cvel, cdof_dot.
"""
function mj_comVel(m, d)
    ccall((:mj_comVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute qfrc_passive from spring-dampers, viscosity and density.
"""
function mj_passive(m, d)
    ccall((:mj_passive, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
subtree linear velocity and angular momentum
"""
function mj_subtreeVel(m, d)
    ccall((:mj_subtreeVel, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.
"""
function mj_rne(m, d, flg_acc, result)
    ccall((:mj_rne, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}), m, d, flg_acc, result)
end
"""
RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
"""
function mj_rnePostConstraint(m, d)
    ccall((:mj_rnePostConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Run collision detection.
"""
function mj_collision(m, d)
    ccall((:mj_collision, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Construct constraints.
"""
function mj_makeConstraint(m, d)
    ccall((:mj_makeConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute inverse constraint inertia efc_AR.
"""
function mj_projectConstraint(m, d)
    ccall((:mj_projectConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute efc_vel, efc_aref.
"""
function mj_referenceConstraint(m, d)
    ccall((:mj_referenceConstraint, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}), m, d)
end
"""
Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians. If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
"""
function mj_constraintUpdate(m, d, jar, cost, flg_coneHessian)
    ccall((:mj_constraintUpdate, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jar, cost, flg_coneHessian)
end
"""
Return size of state specification.
"""
function mj_stateSize(m, spec)
    ccall((:mj_stateSize, libmujoco), Cint, (Ptr{mjModel}, Cuint), m, spec)
end
"""
Get state.
"""
function mj_getState(m, d, state, spec)
    ccall((:mj_getState, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cuint), m, d, state, spec)
end
"""
Set state.
"""
function mj_setState(m, d, state, spec)
    ccall((:mj_setState, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cuint), m, d, state, spec)
end
"""
Add contact to d->contact list; return 0 if success; 1 if buffer full.
"""
function mj_addContact(m, d, con)
    ccall((:mj_addContact, libmujoco), Cint, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjContact}), m, d, con)
end
"""
Determine type of friction cone.
"""
function mj_isPyramidal(m)
    ccall((:mj_isPyramidal, libmujoco), Cint, (Ptr{mjModel},), m)
end
"""
Determine type of constraint Jacobian.
"""
function mj_isSparse(m)
    ccall((:mj_isSparse, libmujoco), Cint, (Ptr{mjModel},), m)
end
"""
Determine type of solver (PGS is dual, CG and Newton are primal).
"""
function mj_isDual(m)
    ccall((:mj_isDual, libmujoco), Cint, (Ptr{mjModel},), m)
end
"""
Multiply dense or sparse constraint Jacobian by vector.
"""
function mj_mulJacVec(m, d, res, vec)
    ccall((:mj_mulJacVec, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
"""
Multiply dense or sparse constraint Jacobian transpose by vector.
"""
function mj_mulJacTVec(m, d, res, vec)
    ccall((:mj_mulJacTVec, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
"""
Compute 3/6-by-nv end-effector Jacobian of global point attached to given body.
"""
function mj_jac(m, d, jacp, jacr, point, body)
    ccall((:mj_jac, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, point, body)
end
"""
Compute body frame end-effector Jacobian.
"""
function mj_jacBody(m, d, jacp, jacr, body)
    ccall((:mj_jacBody, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, body)
end
"""
Compute body center-of-mass end-effector Jacobian.
"""
function mj_jacBodyCom(m, d, jacp, jacr, body)
    ccall((:mj_jacBodyCom, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, body)
end
"""
Compute subtree center-of-mass end-effector Jacobian.
"""
function mj_jacSubtreeCom(m, d, jacp, body)
    ccall((:mj_jacSubtreeCom, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Cint), m, d, jacp, body)
end
"""
Compute geom end-effector Jacobian.
"""
function mj_jacGeom(m, d, jacp, jacr, geom)
    ccall((:mj_jacGeom, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, geom)
end
"""
Compute site end-effector Jacobian.
"""
function mj_jacSite(m, d, jacp, jacr, site)
    ccall((:mj_jacSite, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacp, jacr, site)
end
"""
Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
"""
function mj_jacPointAxis(m, d, jacPoint, jacAxis, point, axis, body)
    ccall((:mj_jacPointAxis, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), m, d, jacPoint, jacAxis, point, axis, body)
end
"""
Get id of object with the specified mjtObj type and name, returns -1 if id not found.
"""
function mj_name2id(m, type, name)
    ccall((:mj_name2id, libmujoco), Cint, (Ptr{mjModel}, Cint, Ptr{Cchar}), m, type, name)
end
"""
Get name of object with the specified mjtObj type and id, returns NULL if name not found.
"""
function mj_id2name(m, type, id)
    ccall((:mj_id2name, libmujoco), Ptr{Cchar}, (Ptr{mjModel}, Cint, Cint), m, type, id)
end
"""
Convert sparse inertia matrix M into full (i.e. dense) matrix.
"""
function mj_fullM(m, dst, M)
    ccall((:mj_fullM, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}), m, dst, M)
end
"""
Multiply vector by inertia matrix.
"""
function mj_mulM(m, d, res, vec)
    ccall((:mj_mulM, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
"""
Multiply vector by (inertia matrix)^(1/2).
"""
function mj_mulM2(m, d, res, vec)
    ccall((:mj_mulM2, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, res, vec)
end
"""
Add inertia matrix to destination matrix. Destination can be sparse uncompressed, or dense when all int* are NULL
"""
function mj_addM(m, d, dst, rownnz, rowadr, colind)
    ccall((:mj_addM, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}, Ptr{Cint}), m, d, dst, rownnz, rowadr, colind)
end
"""
Apply Cartesian force and torque (outside xfrc_applied mechanism).
"""
function mj_applyFT(m, d, force, torque, point, body, qfrc_target)
    ccall((:mj_applyFT, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}), m, d, force, torque, point, body, qfrc_target)
end
"""
Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
"""
function mj_objectVelocity(m, d, objtype, objid, res, flg_local)
    ccall((:mj_objectVelocity, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint), m, d, objtype, objid, res, flg_local)
end
"""
Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation.
"""
function mj_objectAcceleration(m, d, objtype, objid, res, flg_local)
    ccall((:mj_objectAcceleration, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Cint, Ptr{mjtNum}, Cint), m, d, objtype, objid, res, flg_local)
end
"""
Extract 6D force:torque given contact id, in the contact frame.
"""
function mj_contactForce(m, d, id, result)
    ccall((:mj_contactForce, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}), m, d, id, result)
end
"""
Compute velocity by finite-differencing two positions.
"""
function mj_differentiatePos(m, qvel, dt, qpos1, qpos2)
    ccall((:mj_differentiatePos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}), m, qvel, dt, qpos1, qpos2)
end
"""
Integrate position with given velocity.
"""
function mj_integratePos(m, qpos, qvel, dt)
    ccall((:mj_integratePos, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), m, qpos, qvel, dt)
end
"""
Normalize all quaternions in qpos-type vector.
"""
function mj_normalizeQuat(m, qpos)
    ccall((:mj_normalizeQuat, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjtNum}), m, qpos)
end
"""
Map from body local to global Cartesian coordinates.
"""
function mj_local2Global(d, xpos, xmat, pos, quat, body, sameframe)
    ccall((:mj_local2Global, libmujoco), Cvoid, (Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, mjtByte), d, xpos, xmat, pos, quat, body, sameframe)
end
"""
Sum all body masses.
"""
function mj_getTotalmass(m)
    ccall((:mj_getTotalmass, libmujoco), mjtNum, (Ptr{mjModel},), m)
end
"""
Scale body masses and inertias to achieve specified total mass.
"""
function mj_setTotalmass(m, newmass)
    ccall((:mj_setTotalmass, libmujoco), Cvoid, (Ptr{mjModel}, mjtNum), m, newmass)
end
"""
Return a config attribute value of a plugin instance; NULL: invalid plugin instance ID or attribute name
"""
function mj_getPluginConfig(m, plugin_id, attrib)
    ccall((:mj_getPluginConfig, libmujoco), Ptr{Cchar}, (Ptr{mjModel}, Cint, Ptr{Cchar}), m, plugin_id, attrib)
end
"""
Load a dynamic library. The dynamic library is assumed to register one or more plugins.
"""
function mj_loadPluginLibrary(path)
    ccall((:mj_loadPluginLibrary, libmujoco), Cvoid, (Ptr{Cchar},), path)
end
"""
Scan a directory and load all dynamic libraries. Dynamic libraries in the specified directory are assumed to register one or more plugins. Optionally, if a callback is specified, it is called for each dynamic library encountered that registers plugins.
"""
function mj_loadAllPluginLibraries(directory, callback)
    ccall((:mj_loadAllPluginLibraries, libmujoco), Cvoid, (Ptr{Cchar}, mjfPluginLibraryLoadCallback), directory, callback)
end
"""
Return version number: 1.0.2 is encoded as 102.
"""
function mj_version()
    ccall((:mj_version, libmujoco), Cint, ())
end
"""
Return the current version of MuJoCo as a null-terminated string.
"""
function mj_versionString()
    ccall((:mj_versionString, libmujoco), Ptr{Cchar}, ())
end
"""
Intersect multiple rays emanating from a single point. Similar semantics to mj_ray, but vec is an array of (nray x 3) directions.
"""
function mj_multiRay(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid, dist, nray, cutoff)
    ccall((:mj_multiRay, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtByte}, mjtByte, Cint, Ptr{Cint}, Ptr{mjtNum}, Cint, mjtNum), m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid, dist, nray, cutoff)
end
"""
Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude. Return distance (x) to nearest surface, or -1 if no intersection and output geomid. geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.
"""
function mj_ray(m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)
    ccall((:mj_ray, libmujoco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtByte}, mjtByte, Cint, Ptr{Cint}), m, d, pnt, vec, geomgroup, flg_static, bodyexclude, geomid)
end
"""
Intersect ray with hfield, return nearest distance or -1 if no intersection.
"""
function mj_rayHfield(m, d, geomid, pnt, vec)
    ccall((:mj_rayHfield, libmujoco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), m, d, geomid, pnt, vec)
end
"""
Intersect ray with mesh, return nearest distance or -1 if no intersection.
"""
function mj_rayMesh(m, d, geomid, pnt, vec)
    ccall((:mj_rayMesh, libmujoco), mjtNum, (Ptr{mjModel}, Ptr{mjData}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), m, d, geomid, pnt, vec)
end
"""
Intersect ray with pure geom, return nearest distance or -1 if no intersection.
"""
function mju_rayGeom(pos, mat, size, pnt, vec, geomtype)
    ccall((:mju_rayGeom, libmujoco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), pos, mat, size, pnt, vec, geomtype)
end
"""
Intersect ray with skin, return nearest distance or -1 if no intersection, and also output nearest vertex id.
"""
function mju_raySkin(nface, nvert, face, vert, pnt, vec, vertid)
    ccall((:mju_raySkin, libmujoco), mjtNum, (Cint, Cint, Ptr{Cint}, Ptr{Cfloat}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}), nface, nvert, face, vert, pnt, vec, vertid)
end
"""
Set default camera.
"""
function mjv_defaultCamera(cam)
    ccall((:mjv_defaultCamera, libmujoco), Cvoid, (Ptr{mjvCamera},), cam)
end
"""
Set default free camera.
"""
function mjv_defaultFreeCamera(m, cam)
    ccall((:mjv_defaultFreeCamera, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjvCamera}), m, cam)
end
"""
Set default perturbation.
"""
function mjv_defaultPerturb(pert)
    ccall((:mjv_defaultPerturb, libmujoco), Cvoid, (Ptr{mjvPerturb},), pert)
end
"""
Transform pose from room to model space.
"""
function mjv_room2model(modelpos, modelquat, roompos, roomquat, scn)
    ccall((:mjv_room2model, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), modelpos, modelquat, roompos, roomquat, scn)
end
"""
Transform pose from model to room space.
"""
function mjv_model2room(roompos, roomquat, modelpos, modelquat, scn)
    ccall((:mjv_model2room, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), roompos, roomquat, modelpos, modelquat, scn)
end
"""
Get camera info in model space; average left and right OpenGL cameras.
"""
function mjv_cameraInModel(headpos, forward, up, scn)
    ccall((:mjv_cameraInModel, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), headpos, forward, up, scn)
end
"""
Get camera info in room space; average left and right OpenGL cameras.
"""
function mjv_cameraInRoom(headpos, forward, up, scn)
    ccall((:mjv_cameraInRoom, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjvScene}), headpos, forward, up, scn)
end
"""
Get frustum height at unit distance from camera; average left and right OpenGL cameras.
"""
function mjv_frustumHeight(scn)
    ccall((:mjv_frustumHeight, libmujoco), mjtNum, (Ptr{mjvScene},), scn)
end
"""
Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y).
"""
function mjv_alignToCamera(res, vec, forward)
    ccall((:mjv_alignToCamera, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, forward)
end
"""
Move camera with mouse; action is mjtMouse.
"""
function mjv_moveCamera(m, action, reldx, reldy, scn, cam)
    ccall((:mjv_moveCamera, libmujoco), Cvoid, (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}), m, action, reldx, reldy, scn, cam)
end
"""
Move camera with mouse given a scene state; action is mjtMouse.
"""
function mjv_moveCameraFromState(scnstate, action, reldx, reldy, scn, cam)
    ccall((:mjv_moveCameraFromState, libmujoco), Cvoid, (Ptr{mjvSceneState}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvCamera}), scnstate, action, reldx, reldy, scn, cam)
end
"""
Move perturb object with mouse; action is mjtMouse.
"""
function mjv_movePerturb(m, d, action, reldx, reldy, scn, pert)
    ccall((:mjv_movePerturb, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}), m, d, action, reldx, reldy, scn, pert)
end
"""
Move perturb object with mouse given a scene state; action is mjtMouse.
"""
function mjv_movePerturbFromState(scnstate, action, reldx, reldy, scn, pert)
    ccall((:mjv_movePerturbFromState, libmujoco), Cvoid, (Ptr{mjvSceneState}, Cint, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjvPerturb}), scnstate, action, reldx, reldy, scn, pert)
end
"""
Move model with mouse; action is mjtMouse.
"""
function mjv_moveModel(m, action, reldx, reldy, roomup, scn)
    ccall((:mjv_moveModel, libmujoco), Cvoid, (Ptr{mjModel}, Cint, mjtNum, mjtNum, Ptr{mjtNum}, Ptr{mjvScene}), m, action, reldx, reldy, roomup, scn)
end
"""
Copy perturb pos,quat from selected body; set scale for perturbation.
"""
function mjv_initPerturb(m, d, scn, pert)
    ccall((:mjv_initPerturb, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}, Ptr{mjvPerturb}), m, d, scn, pert)
end
"""
Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise. Write d->qpos only if flg_paused and subtree root for selected body has free joint.
"""
function mjv_applyPerturbPose(m, d, pert, flg_paused)
    ccall((:mjv_applyPerturbPose, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}, Cint), m, d, pert, flg_paused)
end
"""
Set perturb force,torque in d->xfrc_applied, if selected body is dynamic.
"""
function mjv_applyPerturbForce(m, d, pert)
    ccall((:mjv_applyPerturbForce, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvPerturb}), m, d, pert)
end
"""
Return the average of two OpenGL cameras.
"""
function mjv_averageCamera(cam1, cam2)
    ccall((:mjv_averageCamera, libmujoco), mjvGLCamera, (Ptr{mjvGLCamera}, Ptr{mjvGLCamera}), cam1, cam2)
end
"""
Select geom or skin with mouse, return bodyid; -1: none selected.
"""
function mjv_select(m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid)
    ccall((:mjv_select, libmujoco), Cint, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, mjtNum, mjtNum, mjtNum, Ptr{mjvScene}, Ptr{mjtNum}, Ptr{Cint}, Ptr{Cint}), m, d, vopt, aspectratio, relx, rely, scn, selpnt, geomid, skinid)
end
"""
Set default visualization options.
"""
function mjv_defaultOption(opt)
    ccall((:mjv_defaultOption, libmujoco), Cvoid, (Ptr{mjvOption},), opt)
end
"""
Set default figure.
"""
function mjv_defaultFigure(fig)
    ccall((:mjv_defaultFigure, libmujoco), Cvoid, (Ptr{mjvFigure},), fig)
end
"""
Initialize given geom fields when not NULL, set the rest to their default values.
"""
function mjv_initGeom(geom, type, size, pos, mat, rgba)
    ccall((:mjv_initGeom, libmujoco), Cvoid, (Ptr{mjvGeom}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cfloat}), geom, type, size, pos, mat, rgba)
end
"""
Set (type, size, pos, mat) for connector-type geom between given points. Assume that mjv_initGeom was already called to set all other properties. Width of mjGEOM_LINE is denominated in pixels. Deprecated: use mjv_connector.
"""
function mjv_makeConnector(geom, type, width, a0, a1, a2, b0, b1, b2)
    ccall((:mjv_makeConnector, libmujoco), Cvoid, (Ptr{mjvGeom}, Cint, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum, mjtNum), geom, type, width, a0, a1, a2, b0, b1, b2)
end
"""
Set (type, size, pos, mat) for connector-type geom between given points. Assume that mjv_initGeom was already called to set all other properties. Width of mjGEOM_LINE is denominated in pixels.
"""
function mjv_connector(geom, type, width, from, to)
    ccall((:mjv_connector, libmujoco), Cvoid, (Ptr{mjvGeom}, Cint, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}), geom, type, width, from, to)
end
"""
Set default abstract scene.
"""
function mjv_defaultScene(scn)
    ccall((:mjv_defaultScene, libmujoco), Cvoid, (Ptr{mjvScene},), scn)
end
"""
Allocate resources in abstract scene.
"""
function mjv_makeScene(m, scn, maxgeom)
    ccall((:mjv_makeScene, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjvScene}, Cint), m, scn, maxgeom)
end
"""
Free abstract scene.
"""
function mjv_freeScene(scn)
    ccall((:mjv_freeScene, libmujoco), Cvoid, (Ptr{mjvScene},), scn)
end
"""
Update entire scene given model state.
"""
function mjv_updateScene(m, d, opt, pert, cam, catmask, scn)
    ccall((:mjv_updateScene, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Ptr{mjvCamera}, Cint, Ptr{mjvScene}), m, d, opt, pert, cam, catmask, scn)
end
"""
Update entire scene from a scene state, return the number of new mjWARN_VGEOMFULL warnings.
"""
function mjv_updateSceneFromState(scnstate, opt, pert, cam, catmask, scn)
    ccall((:mjv_updateSceneFromState, libmujoco), Cint, (Ptr{mjvSceneState}, Ptr{mjvOption}, Ptr{mjvPerturb}, Ptr{mjvCamera}, Cint, Ptr{mjvScene}), scnstate, opt, pert, cam, catmask, scn)
end
"""
Set default scene state.
"""
function mjv_defaultSceneState(scnstate)
    ccall((:mjv_defaultSceneState, libmujoco), Cvoid, (Ptr{mjvSceneState},), scnstate)
end
"""
Allocate resources and initialize a scene state object.
"""
function mjv_makeSceneState(m, d, scnstate, maxgeom)
    ccall((:mjv_makeSceneState, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvSceneState}, Cint), m, d, scnstate, maxgeom)
end
"""
Free scene state.
"""
function mjv_freeSceneState(scnstate)
    ccall((:mjv_freeSceneState, libmujoco), Cvoid, (Ptr{mjvSceneState},), scnstate)
end
"""
Update a scene state from model and data.
"""
function mjv_updateSceneState(m, d, opt, scnstate)
    ccall((:mjv_updateSceneState, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvSceneState}), m, d, opt, scnstate)
end
"""
Add geoms from selected categories.
"""
function mjv_addGeoms(m, d, opt, pert, catmask, scn)
    ccall((:mjv_addGeoms, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvOption}, Ptr{mjvPerturb}, Cint, Ptr{mjvScene}), m, d, opt, pert, catmask, scn)
end
"""
Make list of lights.
"""
function mjv_makeLights(m, d, scn)
    ccall((:mjv_makeLights, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}), m, d, scn)
end
"""
Update camera.
"""
function mjv_updateCamera(m, d, cam, scn)
    ccall((:mjv_updateCamera, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvCamera}, Ptr{mjvScene}), m, d, cam, scn)
end
"""
Update skins.
"""
function mjv_updateSkin(m, d, scn)
    ccall((:mjv_updateSkin, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, Ptr{mjvScene}), m, d, scn)
end
"""
Set default mjrContext.
"""
function mjr_defaultContext(con)
    ccall((:mjr_defaultContext, libmujoco), Cvoid, (Ptr{mjrContext},), con)
end
"""
Allocate resources in custom OpenGL context; fontscale is mjtFontScale.
"""
function mjr_makeContext(m, con, fontscale)
    ccall((:mjr_makeContext, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, fontscale)
end
"""
Change font of existing context.
"""
function mjr_changeFont(fontscale, con)
    ccall((:mjr_changeFont, libmujoco), Cvoid, (Cint, Ptr{mjrContext}), fontscale, con)
end
"""
Add Aux buffer with given index to context; free previous Aux buffer.
"""
function mjr_addAux(index, width, height, samples, con)
    ccall((:mjr_addAux, libmujoco), Cvoid, (Cint, Cint, Cint, Cint, Ptr{mjrContext}), index, width, height, samples, con)
end
"""
Free resources in custom OpenGL context, set to default.
"""
function mjr_freeContext(con)
    ccall((:mjr_freeContext, libmujoco), Cvoid, (Ptr{mjrContext},), con)
end
"""
Resize offscreen buffers.
"""
function mjr_resizeOffscreen(width, height, con)
    ccall((:mjr_resizeOffscreen, libmujoco), Cvoid, (Cint, Cint, Ptr{mjrContext}), width, height, con)
end
"""
Upload texture to GPU, overwriting previous upload if any.
"""
function mjr_uploadTexture(m, con, texid)
    ccall((:mjr_uploadTexture, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, texid)
end
"""
Upload mesh to GPU, overwriting previous upload if any.
"""
function mjr_uploadMesh(m, con, meshid)
    ccall((:mjr_uploadMesh, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, meshid)
end
"""
Upload height field to GPU, overwriting previous upload if any.
"""
function mjr_uploadHField(m, con, hfieldid)
    ccall((:mjr_uploadHField, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjrContext}, Cint), m, con, hfieldid)
end
"""
Make con->currentBuffer current again.
"""
function mjr_restoreBuffer(con)
    ccall((:mjr_restoreBuffer, libmujoco), Cvoid, (Ptr{mjrContext},), con)
end
"""
Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN. If only one buffer is available, set that buffer and ignore framebuffer argument.
"""
function mjr_setBuffer(framebuffer, con)
    ccall((:mjr_setBuffer, libmujoco), Cvoid, (Cint, Ptr{mjrContext}), framebuffer, con)
end
"""
Read pixels from current OpenGL framebuffer to client buffer. Viewport is in OpenGL framebuffer; client buffer starts at (0,0).
"""
function mjr_readPixels(rgb, depth, viewport, con)
    ccall((:mjr_readPixels, libmujoco), Cvoid, (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}), rgb, depth, viewport, con)
end
"""
Draw pixels from client buffer to current OpenGL framebuffer. Viewport is in OpenGL framebuffer; client buffer starts at (0,0).
"""
function mjr_drawPixels(rgb, depth, viewport, con)
    ccall((:mjr_drawPixels, libmujoco), Cvoid, (Ptr{Cuchar}, Ptr{Cfloat}, mjrRect, Ptr{mjrContext}), rgb, depth, viewport, con)
end
"""
Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer. If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR.
"""
function mjr_blitBuffer(src, dst, flg_color, flg_depth, con)
    ccall((:mjr_blitBuffer, libmujoco), Cvoid, (mjrRect, mjrRect, Cint, Cint, Ptr{mjrContext}), src, dst, flg_color, flg_depth, con)
end
"""
Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
"""
function mjr_setAux(index, con)
    ccall((:mjr_setAux, libmujoco), Cvoid, (Cint, Ptr{mjrContext}), index, con)
end
"""
Blit from Aux buffer to con->currentBuffer.
"""
function mjr_blitAux(index, src, left, bottom, con)
    ccall((:mjr_blitAux, libmujoco), Cvoid, (Cint, mjrRect, Cint, Cint, Ptr{mjrContext}), index, src, left, bottom, con)
end
"""
Draw text at (x,y) in relative coordinates; font is mjtFont.
"""
function mjr_text(font, txt, con, x, y, r, g, b)
    ccall((:mjr_text, libmujoco), Cvoid, (Cint, Ptr{Cchar}, Ptr{mjrContext}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat), font, txt, con, x, y, r, g, b)
end
"""
Draw text overlay; font is mjtFont; gridpos is mjtGridPos.
"""
function mjr_overlay(font, gridpos, viewport, overlay, overlay2, con)
    ccall((:mjr_overlay, libmujoco), Cvoid, (Cint, Cint, mjrRect, Ptr{Cchar}, Ptr{Cchar}, Ptr{mjrContext}), font, gridpos, viewport, overlay, overlay2, con)
end
"""
Get maximum viewport for active buffer.
"""
function mjr_maxViewport(con)
    ccall((:mjr_maxViewport, libmujoco), mjrRect, (Ptr{mjrContext},), con)
end
"""
Draw rectangle.
"""
function mjr_rectangle(viewport, r, g, b, a)
    ccall((:mjr_rectangle, libmujoco), Cvoid, (mjrRect, Cfloat, Cfloat, Cfloat, Cfloat), viewport, r, g, b, a)
end
"""
Draw rectangle with centered text.
"""
function mjr_label(viewport, font, txt, r, g, b, a, rt, gt, bt, con)
    ccall((:mjr_label, libmujoco), Cvoid, (mjrRect, Cint, Ptr{Cchar}, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Cfloat, Ptr{mjrContext}), viewport, font, txt, r, g, b, a, rt, gt, bt, con)
end
"""
Draw 2D figure.
"""
function mjr_figure(viewport, fig, con)
    ccall((:mjr_figure, libmujoco), Cvoid, (mjrRect, Ptr{mjvFigure}, Ptr{mjrContext}), viewport, fig, con)
end
"""
Render 3D scene.
"""
function mjr_render(viewport, scn, con)
    ccall((:mjr_render, libmujoco), Cvoid, (mjrRect, Ptr{mjvScene}, Ptr{mjrContext}), viewport, scn, con)
end
"""
Call glFinish.
"""
function mjr_finish()
    ccall((:mjr_finish, libmujoco), Cvoid, ())
end
"""
Call glGetError and return result.
"""
function mjr_getError()
    ccall((:mjr_getError, libmujoco), Cint, ())
end
"""
Find first rectangle containing mouse, -1: not found.
"""
function mjr_findRect(x, y, nrect, rect)
    ccall((:mjr_findRect, libmujoco), Cint, (Cint, Cint, Cint, Ptr{mjrRect}), x, y, nrect, rect)
end
"""
Get builtin UI theme spacing (ind: 0-1).
"""
function mjui_themeSpacing(ind)
    ccall((:mjui_themeSpacing, libmujoco), mjuiThemeSpacing, (Cint,), ind)
end
"""
Get builtin UI theme color (ind: 0-3).
"""
function mjui_themeColor(ind)
    ccall((:mjui_themeColor, libmujoco), mjuiThemeColor, (Cint,), ind)
end
"""
Add definitions to UI.
"""
function mjui_add(ui, def)
    ccall((:mjui_add, libmujoco), Cvoid, (Ptr{mjUI}, Ptr{mjuiDef}), ui, def)
end
"""
Add definitions to UI section.
"""
function mjui_addToSection(ui, sect, def)
    ccall((:mjui_addToSection, libmujoco), Cvoid, (Ptr{mjUI}, Cint, Ptr{mjuiDef}), ui, sect, def)
end
"""
Compute UI sizes.
"""
function mjui_resize(ui, con)
    ccall((:mjui_resize, libmujoco), Cvoid, (Ptr{mjUI}, Ptr{mjrContext}), ui, con)
end
"""
Update specific section/item; -1: update all.
"""
function mjui_update(section, item, ui, state, con)
    ccall((:mjui_update, libmujoco), Cvoid, (Cint, Cint, Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), section, item, ui, state, con)
end
"""
Handle UI event, return pointer to changed item, NULL if no change.
"""
function mjui_event(ui, state, con)
    ccall((:mjui_event, libmujoco), Ptr{mjuiItem}, (Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), ui, state, con)
end
"""
Copy UI image to current buffer.
"""
function mjui_render(ui, state, con)
    ccall((:mjui_render, libmujoco), Cvoid, (Ptr{mjUI}, Ptr{mjuiState}, Ptr{mjrContext}), ui, state, con)
end
"""
Deprecated: use mju_error.
"""
function mju_error_i(msg, i)
    ccall((:mju_error_i, libmujoco), Cvoid, (Ptr{Cchar}, Cint), msg, i)
end
"""
Deprecated: use mju_error.
"""
function mju_error_s(msg, text)
    ccall((:mju_error_s, libmujoco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), msg, text)
end
"""
Deprecated: use mju_warning.
"""
function mju_warning_i(msg, i)
    ccall((:mju_warning_i, libmujoco), Cvoid, (Ptr{Cchar}, Cint), msg, i)
end
"""
Deprecated: use mju_warning.
"""
function mju_warning_s(msg, text)
    ccall((:mju_warning_s, libmujoco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), msg, text)
end
"""
Clear user error and memory handlers.
"""
function mju_clearHandlers()
    ccall((:mju_clearHandlers, libmujoco), Cvoid, ())
end
"""
Allocate memory; byte-align on 64; pad size to multiple of 64.
"""
function mju_malloc(size)
    ccall((:mju_malloc, libmujoco), Ptr{Cvoid}, (Csize_t,), size)
end
"""
Free memory, using free() by default.
"""
function mju_free(ptr)
    ccall((:mju_free, libmujoco), Cvoid, (Ptr{Cvoid},), ptr)
end
"""
High-level warning function: count warnings in mjData, print only the first.
"""
function mj_warning(d, warning, info)
    ccall((:mj_warning, libmujoco), Cvoid, (Ptr{mjData}, Cint, Cint), d, warning, info)
end
"""
Write [datetime, type: message] to MUJOCO_LOG.TXT.
"""
function mju_writeLog(type, msg)
    ccall((:mju_writeLog, libmujoco), Cvoid, (Ptr{Cchar}, Ptr{Cchar}), type, msg)
end
"""
Set res = 0.
"""
function mju_zero3(res)
    ccall((:mju_zero3, libmujoco), Cvoid, (Ptr{mjtNum},), res)
end
"""
Set res = vec.
"""
function mju_copy3(res, data)
    ccall((:mju_copy3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, data)
end
"""
Set res = vec*scl.
"""
function mju_scl3(res, vec, scl)
    ccall((:mju_scl3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec, scl)
end
"""
Set res = vec1 + vec2.
"""
function mju_add3(res, vec1, vec2)
    ccall((:mju_add3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec1, vec2)
end
"""
Set res = vec1 - vec2.
"""
function mju_sub3(res, vec1, vec2)
    ccall((:mju_sub3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec1, vec2)
end
"""
Set res = res + vec.
"""
function mju_addTo3(res, vec)
    ccall((:mju_addTo3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, vec)
end
"""
Set res = res - vec.
"""
function mju_subFrom3(res, vec)
    ccall((:mju_subFrom3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, vec)
end
"""
Set res = res + vec*scl.
"""
function mju_addToScl3(res, vec, scl)
    ccall((:mju_addToScl3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec, scl)
end
"""
Set res = vec1 + vec2*scl.
"""
function mju_addScl3(res, vec1, vec2, scl)
    ccall((:mju_addScl3, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, vec1, vec2, scl)
end
"""
Normalize vector, return length before normalization.
"""
function mju_normalize3(res)
    ccall((:mju_normalize3, libmujoco), mjtNum, (Ptr{mjtNum},), res)
end
"""
Return vector length (without normalizing the vector).
"""
function mju_norm3(vec)
    ccall((:mju_norm3, libmujoco), mjtNum, (Ptr{mjtNum},), vec)
end
"""
Return dot-product of vec1 and vec2.
"""
function mju_dot3(vec1, vec2)
    ccall((:mju_dot3, libmujoco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}), vec1, vec2)
end
"""
Return Cartesian distance between 3D vectors pos1 and pos2.
"""
function mju_dist3(pos1, pos2)
    ccall((:mju_dist3, libmujoco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}), pos1, pos2)
end
"""
Multiply vector by 3D rotation matrix: res = mat * vec.
"""
function mju_rotVecMat(res, vec, mat)
    ccall((:mju_rotVecMat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, mat)
end
"""
Multiply vector by transposed 3D rotation matrix: res = mat' * vec.
"""
function mju_rotVecMatT(res, vec, mat)
    ccall((:mju_rotVecMatT, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, mat)
end
"""
Compute cross-product: res = cross(a, b).
"""
function mju_cross(res, a, b)
    ccall((:mju_cross, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, a, b)
end
"""
Set res = 0.
"""
function mju_zero4(res)
    ccall((:mju_zero4, libmujoco), Cvoid, (Ptr{mjtNum},), res)
end
"""
Set res = (1,0,0,0).
"""
function mju_unit4(res)
    ccall((:mju_unit4, libmujoco), Cvoid, (Ptr{mjtNum},), res)
end
"""
Set res = vec.
"""
function mju_copy4(res, data)
    ccall((:mju_copy4, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, data)
end
"""
Normalize vector, return length before normalization.
"""
function mju_normalize4(res)
    ccall((:mju_normalize4, libmujoco), mjtNum, (Ptr{mjtNum},), res)
end
"""
Set res = 0.
"""
function mju_zero(res, n)
    ccall((:mju_zero, libmujoco), Cvoid, (Ptr{mjtNum}, Cint), res, n)
end
"""
Set res = val.
"""
function mju_fill(res, val, n)
    ccall((:mju_fill, libmujoco), Cvoid, (Ptr{mjtNum}, mjtNum, Cint), res, val, n)
end
"""
Set res = vec.
"""
function mju_copy(res, data, n)
    ccall((:mju_copy, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, data, n)
end
"""
Return sum(vec).
"""
function mju_sum(vec, n)
    ccall((:mju_sum, libmujoco), mjtNum, (Ptr{mjtNum}, Cint), vec, n)
end
"""
Return L1 norm: sum(abs(vec)).
"""
function mju_L1(vec, n)
    ccall((:mju_L1, libmujoco), mjtNum, (Ptr{mjtNum}, Cint), vec, n)
end
"""
Set res = vec*scl.
"""
function mju_scl(res, vec, scl, n)
    ccall((:mju_scl, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec, scl, n)
end
"""
Set res = vec1 + vec2.
"""
function mju_add(res, vec1, vec2, n)
    ccall((:mju_add, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec1, vec2, n)
end
"""
Set res = vec1 - vec2.
"""
function mju_sub(res, vec1, vec2, n)
    ccall((:mju_sub, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec1, vec2, n)
end
"""
Set res = res + vec.
"""
function mju_addTo(res, vec, n)
    ccall((:mju_addTo, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec, n)
end
"""
Set res = res - vec.
"""
function mju_subFrom(res, vec, n)
    ccall((:mju_subFrom, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, vec, n)
end
"""
Set res = res + vec*scl.
"""
function mju_addToScl(res, vec, scl, n)
    ccall((:mju_addToScl, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec, scl, n)
end
"""
Set res = vec1 + vec2*scl.
"""
function mju_addScl(res, vec1, vec2, scl, n)
    ccall((:mju_addScl, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, mjtNum, Cint), res, vec1, vec2, scl, n)
end
"""
Normalize vector, return length before normalization.
"""
function mju_normalize(res, n)
    ccall((:mju_normalize, libmujoco), mjtNum, (Ptr{mjtNum}, Cint), res, n)
end
"""
Return vector length (without normalizing vector).
"""
function mju_norm(res, n)
    ccall((:mju_norm, libmujoco), mjtNum, (Ptr{mjtNum}, Cint), res, n)
end
"""
Return dot-product of vec1 and vec2.
"""
function mju_dot(vec1, vec2, n)
    ccall((:mju_dot, libmujoco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), vec1, vec2, n)
end
"""
Multiply matrix and vector: res = mat * vec.
"""
function mju_mulMatVec(res, mat, vec, nr, nc)
    ccall((:mju_mulMatVec, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, vec, nr, nc)
end
"""
Multiply transposed matrix and vector: res = mat' * vec.
"""
function mju_mulMatTVec(res, mat, vec, nr, nc)
    ccall((:mju_mulMatTVec, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, vec, nr, nc)
end
"""
Multiply square matrix with vectors on both sides: returns vec1' * mat * vec2.
"""
function mju_mulVecMatVec(vec1, mat, vec2, n)
    ccall((:mju_mulVecMatVec, libmujoco), mjtNum, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), vec1, mat, vec2, n)
end
"""
Transpose matrix: res = mat'.
"""
function mju_transpose(res, mat, nr, nc)
    ccall((:mju_transpose, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, nr, nc)
end
"""
Symmetrize square matrix res = (mat + mat')/2.
"""
function mju_symmetrize(res, mat, n)
    ccall((:mju_symmetrize, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, mat, n)
end
"""
Set mat to the identity matrix.
"""
function mju_eye(mat, n)
    ccall((:mju_eye, libmujoco), Cvoid, (Ptr{mjtNum}, Cint), mat, n)
end
"""
Multiply matrices: res = mat1 * mat2.
"""
function mju_mulMatMat(res, mat1, mat2, r1, c1, c2)
    ccall((:mju_mulMatMat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, c2)
end
"""
Multiply matrices, second argument transposed: res = mat1 * mat2'.
"""
function mju_mulMatMatT(res, mat1, mat2, r1, c1, r2)
    ccall((:mju_mulMatMatT, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, r2)
end
"""
Multiply matrices, first argument transposed: res = mat1' * mat2.
"""
function mju_mulMatTMat(res, mat1, mat2, r1, c1, c2)
    ccall((:mju_mulMatTMat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat1, mat2, r1, c1, c2)
end
"""
Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise.
"""
function mju_sqrMatTD(res, mat, diag, nr, nc)
    ccall((:mju_sqrMatTD, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), res, mat, diag, nr, nc)
end
"""
Coordinate transform of 6D motion or force vector in rotation:translation format. rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type.
"""
function mju_transformSpatial(res, vec, flg_force, newpos, oldpos, rotnew2old)
    ccall((:mju_transformSpatial, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, flg_force, newpos, oldpos, rotnew2old)
end
"""
Rotate vector by quaternion.
"""
function mju_rotVecQuat(res, vec, quat)
    ccall((:mju_rotVecQuat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, vec, quat)
end
"""
Conjugate quaternion, corresponding to opposite rotation.
"""
function mju_negQuat(res, quat)
    ccall((:mju_negQuat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, quat)
end
"""
Multiply quaternions.
"""
function mju_mulQuat(res, quat1, quat2)
    ccall((:mju_mulQuat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat1, quat2)
end
"""
Multiply quaternion and axis.
"""
function mju_mulQuatAxis(res, quat, axis)
    ccall((:mju_mulQuatAxis, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat, axis)
end
"""
Convert axisAngle to quaternion.
"""
function mju_axisAngle2Quat(res, axis, angle)
    ccall((:mju_axisAngle2Quat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, axis, angle)
end
"""
Convert quaternion (corresponding to orientation difference) to 3D velocity.
"""
function mju_quat2Vel(res, quat, dt)
    ccall((:mju_quat2Vel, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), res, quat, dt)
end
"""
Subtract quaternions, express as 3D velocity: qb*quat(res) = qa.
"""
function mju_subQuat(res, qa, qb)
    ccall((:mju_subQuat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, qa, qb)
end
"""
Convert quaternion to 3D rotation matrix.
"""
function mju_quat2Mat(res, quat)
    ccall((:mju_quat2Mat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), res, quat)
end
"""
Convert 3D rotation matrix to quaternion.
"""
function mju_mat2Quat(quat, mat)
    ccall((:mju_mat2Quat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), quat, mat)
end
"""
Compute time-derivative of quaternion, given 3D rotational velocity.
"""
function mju_derivQuat(res, quat, vel)
    ccall((:mju_derivQuat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, quat, vel)
end
"""
Integrate quaternion given 3D angular velocity.
"""
function mju_quatIntegrate(quat, vel, scale)
    ccall((:mju_quatIntegrate, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, mjtNum), quat, vel, scale)
end
"""
Construct quaternion performing rotation from z-axis to given vector.
"""
function mju_quatZ2Vec(quat, vec)
    ccall((:mju_quatZ2Vec, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}), quat, vec)
end
"""
Multiply two poses.
"""
function mju_mulPose(posres, quatres, pos1, quat1, pos2, quat2)
    ccall((:mju_mulPose, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), posres, quatres, pos1, quat1, pos2, quat2)
end
"""
Conjugate pose, corresponding to the opposite spatial transformation.
"""
function mju_negPose(posres, quatres, pos, quat)
    ccall((:mju_negPose, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), posres, quatres, pos, quat)
end
"""
Transform vector by pose.
"""
function mju_trnVecPose(res, pos, quat, vec)
    ccall((:mju_trnVecPose, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), res, pos, quat, vec)
end
"""
Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat.
"""
function mju_cholFactor(mat, n, mindiag)
    ccall((:mju_cholFactor, libmujoco), Cint, (Ptr{mjtNum}, Cint, mjtNum), mat, n, mindiag)
end
"""
Solve (mat*mat') * res = vec, where mat is a Cholesky factor.
"""
function mju_cholSolve(res, mat, vec, n)
    ccall((:mju_cholSolve, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), res, mat, vec, n)
end
"""
Cholesky rank-one update: L*L' +/- x*x'; return rank.
"""
function mju_cholUpdate(mat, x, n, flg_plus)
    ccall((:mju_cholUpdate, libmujoco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint), mat, x, n, flg_plus)
end
"""
Band-dense Cholesky decomposition.  Returns minimum value in the factorized diagonal, or 0 if rank-deficient.  mat has (ntotal-ndense) x nband + ndense x ntotal elements.  The first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive.  The second ndense x ntotal store the band part as entire dense rows.  Add diagadd+diagmul*mat_ii to diagonal before factorization.
"""
function mju_cholFactorBand(mat, ntotal, nband, ndense, diagadd, diagmul)
    ccall((:mju_cholFactorBand, libmujoco), mjtNum, (Ptr{mjtNum}, Cint, Cint, Cint, mjtNum, mjtNum), mat, ntotal, nband, ndense, diagadd, diagmul)
end
"""
Solve (mat*mat')*res = vec where mat is a band-dense Cholesky factor.
"""
function mju_cholSolveBand(res, mat, vec, ntotal, nband, ndense)
    ccall((:mju_cholSolveBand, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat, vec, ntotal, nband, ndense)
end
"""
Convert banded matrix to dense matrix, fill upper triangle if flg_sym>0.
"""
function mju_band2Dense(res, mat, ntotal, nband, ndense, flg_sym)
    ccall((:mju_band2Dense, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint, mjtByte), res, mat, ntotal, nband, ndense, flg_sym)
end
"""
Convert dense matrix to banded matrix.
"""
function mju_dense2Band(res, mat, ntotal, nband, ndense)
    ccall((:mju_dense2Band, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint), res, mat, ntotal, nband, ndense)
end
"""
Multiply band-diagonal matrix with nvec vectors, include upper triangle if flg_sym>0.
"""
function mju_bandMulMatVec(res, mat, vec, ntotal, nband, ndense, nvec, flg_sym)
    ccall((:mju_bandMulMatVec, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Cint, Cint, Cint, mjtByte), res, mat, vec, ntotal, nband, ndense, nvec, flg_sym)
end
"""
Address of diagonal element i in band-dense matrix representation.
"""
function mju_bandDiag(i, ntotal, nband, ndense)
    ccall((:mju_bandDiag, libmujoco), Cint, (Cint, Cint, Cint, Cint), i, ntotal, nband, ndense)
end
"""
Eigenvalue decomposition of symmetric 3x3 matrix.
"""
function mju_eig3(eigval, eigvec, quat, mat)
    ccall((:mju_eig3, libmujoco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), eigval, eigvec, quat, mat)
end
"""
minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <= upper, return rank or -1 if failed   inputs:     n           - problem dimension     H           - SPD matrix                n*n     g           - bias vector               n     lower       - lower bounds              n     upper       - upper bounds              n     res         - solution warmstart        n   return value:     nfree <= n  - rank of unconstrained subspace, -1 if failure   outputs (required):     res         - solution                  n     R           - subspace Cholesky factor  nfree*nfree    allocated: n*(n+7)   outputs (optional):     index       - set of free dimensions    nfree          allocated: n   notes:     the initial value of res is used to warmstart the solver     R must have allocatd size n*(n+7), but only nfree*nfree values are used in output     index (if given) must have allocated size n, but only nfree values are used in output     only the lower triangles of H and R and are read from and written to, respectively     the convenience function mju_boxQPmalloc allocates the required data structures
"""
function mju_boxQP(res, R, index, H, g, n, lower, upper)
    ccall((:mju_boxQP, libmujoco), Cint, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{Cint}, Ptr{mjtNum}, Ptr{mjtNum}, Cint, Ptr{mjtNum}, Ptr{mjtNum}), res, R, index, H, g, n, lower, upper)
end
"""
allocate heap memory for box-constrained Quadratic Program   as in mju_boxQP, index, lower, and upper are optional   free all pointers with mju_free()
"""
function mju_boxQPmalloc(res, R, index, H, g, n, lower, upper)
    ccall((:mju_boxQPmalloc, libmujoco), Cvoid, (Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}, Ptr{Ptr{Cint}}, Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}, Cint, Ptr{Ptr{mjtNum}}, Ptr{Ptr{mjtNum}}), res, R, index, H, g, n, lower, upper)
end
"""
Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
"""
function mju_muscleGain(len, vel, lengthrange, acc0, prm)
    ccall((:mju_muscleGain, libmujoco), mjtNum, (mjtNum, mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}), len, vel, lengthrange, acc0, prm)
end
"""
Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax).
"""
function mju_muscleBias(len, lengthrange, acc0, prm)
    ccall((:mju_muscleBias, libmujoco), mjtNum, (mjtNum, Ptr{mjtNum}, mjtNum, Ptr{mjtNum}), len, lengthrange, acc0, prm)
end
"""
Muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width).
"""
function mju_muscleDynamics(ctrl, act, prm)
    ccall((:mju_muscleDynamics, libmujoco), mjtNum, (mjtNum, mjtNum, Ptr{mjtNum}), ctrl, act, prm)
end
"""
Convert contact force to pyramid representation.
"""
function mju_encodePyramid(pyramid, force, mu, dim)
    ccall((:mju_encodePyramid, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), pyramid, force, mu, dim)
end
"""
Convert pyramid representation to contact force.
"""
function mju_decodePyramid(force, pyramid, mu, dim)
    ccall((:mju_decodePyramid, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Cint), force, pyramid, mu, dim)
end
"""
Integrate spring-damper analytically, return pos(dt).
"""
function mju_springDamper(pos0, vel0, Kp, Kv, dt)
    ccall((:mju_springDamper, libmujoco), mjtNum, (mjtNum, mjtNum, mjtNum, mjtNum, mjtNum), pos0, vel0, Kp, Kv, dt)
end
"""
Return min(a,b) with single evaluation of a and b.
"""
function mju_min(a, b)
    ccall((:mju_min, libmujoco), mjtNum, (mjtNum, mjtNum), a, b)
end
"""
Return max(a,b) with single evaluation of a and b.
"""
function mju_max(a, b)
    ccall((:mju_max, libmujoco), mjtNum, (mjtNum, mjtNum), a, b)
end
"""
Clip x to the range [min, max].
"""
function mju_clip(x, min, max)
    ccall((:mju_clip, libmujoco), mjtNum, (mjtNum, mjtNum, mjtNum), x, min, max)
end
"""
Return sign of x: +1, -1 or 0.
"""
function mju_sign(x)
    ccall((:mju_sign, libmujoco), mjtNum, (mjtNum,), x)
end
"""
Round x to nearest integer.
"""
function mju_round(x)
    ccall((:mju_round, libmujoco), Cint, (mjtNum,), x)
end
"""
Convert type id (mjtObj) to type name.
"""
function mju_type2Str(type)
    ccall((:mju_type2Str, libmujoco), Ptr{Cchar}, (Cint,), type)
end
"""
Convert type name to type id (mjtObj).
"""
function mju_str2Type(str)
    ccall((:mju_str2Type, libmujoco), Cint, (Ptr{Cchar},), str)
end
"""
Return human readable number of bytes using standard letter suffix.
"""
function mju_writeNumBytes(nbytes)
    ccall((:mju_writeNumBytes, libmujoco), Ptr{Cchar}, (Csize_t,), nbytes)
end
"""
Construct a warning message given the warning type and info.
"""
function mju_warningText(warning, info)
    ccall((:mju_warningText, libmujoco), Ptr{Cchar}, (Cint, Csize_t), warning, info)
end
"""
Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions.
"""
function mju_isBad(x)
    ccall((:mju_isBad, libmujoco), Cint, (mjtNum,), x)
end
"""
Return 1 if all elements are 0.
"""
function mju_isZero(vec, n)
    ccall((:mju_isZero, libmujoco), Cint, (Ptr{mjtNum}, Cint), vec, n)
end
"""
Standard normal random number generator (optional second number).
"""
function mju_standardNormal(num2)
    ccall((:mju_standardNormal, libmujoco), mjtNum, (Ptr{mjtNum},), num2)
end
"""
Convert from float to mjtNum.
"""
function mju_f2n(res, vec, n)
    ccall((:mju_f2n, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{Cfloat}, Cint), res, vec, n)
end
"""
Convert from mjtNum to float.
"""
function mju_n2f(res, vec, n)
    ccall((:mju_n2f, libmujoco), Cvoid, (Ptr{Cfloat}, Ptr{mjtNum}, Cint), res, vec, n)
end
"""
Convert from double to mjtNum.
"""
function mju_d2n(res, vec, n)
    ccall((:mju_d2n, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{Cdouble}, Cint), res, vec, n)
end
"""
Convert from mjtNum to double.
"""
function mju_n2d(res, vec, n)
    ccall((:mju_n2d, libmujoco), Cvoid, (Ptr{Cdouble}, Ptr{mjtNum}, Cint), res, vec, n)
end
"""
Insertion sort, resulting list is in increasing order.
"""
function mju_insertionSort(list, n)
    ccall((:mju_insertionSort, libmujoco), Cvoid, (Ptr{mjtNum}, Cint), list, n)
end
"""
Integer insertion sort, resulting list is in increasing order.
"""
function mju_insertionSortInt(list, n)
    ccall((:mju_insertionSortInt, libmujoco), Cvoid, (Ptr{Cint}, Cint), list, n)
end
"""
Generate Halton sequence.
"""
function mju_Halton(index, base)
    ccall((:mju_Halton, libmujoco), mjtNum, (Cint, Cint), index, base)
end
"""
Call strncpy, then set dst[n-1] = 0.
"""
function mju_strncpy(dst, src, n)
    ccall((:mju_strncpy, libmujoco), Ptr{Cchar}, (Ptr{Cchar}, Ptr{Cchar}, Cint), dst, src, n)
end
"""
Sigmoid function over 0<=x<=1 using quintic polynomial.
"""
function mju_sigmoid(x)
    ccall((:mju_sigmoid, libmujoco), mjtNum, (mjtNum,), x)
end
"""
Finite differenced transition matrices (control theory notation)   d(x_next) = A*dx + B*du   d(sensor) = C*dx + D*du   required output matrix dimensions:      A: (2*nv+na x 2*nv+na)      B: (2*nv+na x nu)      D: (nsensordata x 2*nv+na)      C: (nsensordata x nu)
"""
function mjd_transitionFD(m, d, eps, flg_centered, A, B, C, D)
    ccall((:mjd_transitionFD, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, mjtNum, mjtByte, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, eps, flg_centered, A, B, C, D)
end
"""
Finite differenced Jacobians of (force, sensors) = mj_inverse(state, acceleration)   All outputs are optional. Output dimensions (transposed w.r.t Control Theory convention):     DfDq: (nv x nv)     DfDv: (nv x nv)     DfDa: (nv x nv)     DsDq: (nv x nsensordata)     DsDv: (nv x nsensordata)     DsDa: (nv x nsensordata)     DmDq: (nv x nM)   single-letter shortcuts:     inputs: q=qpos, v=qvel, a=qacc     outputs: f=qfrc_inverse, s=sensordata, m=qM   notes:     optionally computes mass matrix Jacobian DmDq     flg_actuation specifies whether to subtract qfrc_actuator from qfrc_inverse
"""
function mjd_inverseFD(m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)
    ccall((:mjd_inverseFD, libmujoco), Cvoid, (Ptr{mjModel}, Ptr{mjData}, mjtNum, mjtByte, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), m, d, eps, flg_actuation, DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq)
end
"""
Derivatives of mju_subQuat.
"""
function mjd_subQuat(qa, qb, Da, Db)
    ccall((:mjd_subQuat, libmujoco), Cvoid, (Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), qa, qb, Da, Db)
end
"""
Derivatives of mju_quatIntegrate.
"""
function mjd_quatIntegrate(vel, scale, Dquat, Dvel, Dscale)
    ccall((:mjd_quatIntegrate, libmujoco), Cvoid, (Ptr{mjtNum}, mjtNum, Ptr{mjtNum}, Ptr{mjtNum}, Ptr{mjtNum}), vel, scale, Dquat, Dvel, Dscale)
end
"""
Set default plugin definition.
"""
function mjp_defaultPlugin(plugin)
    ccall((:mjp_defaultPlugin, libmujoco), Cvoid, (Ptr{mjpPlugin},), plugin)
end
"""
Globally register a plugin. This function is thread-safe. If an identical mjpPlugin is already registered, this function does nothing. If a non-identical mjpPlugin with the same name is already registered, an mju_error is raised. Two mjpPlugins are considered identical if all member function pointers and numbers are equal, and the name and attribute strings are all identical, however the char pointers to the strings need not be the same.
"""
function mjp_registerPlugin(plugin)
    ccall((:mjp_registerPlugin, libmujoco), Cint, (Ptr{mjpPlugin},), plugin)
end
"""
Return the number of globally registered plugins.
"""
function mjp_pluginCount()
    ccall((:mjp_pluginCount, libmujoco), Cint, ())
end
"""
Look up a plugin by name. If slot is not NULL, also write its registered slot number into it.
"""
function mjp_getPlugin(name, slot)
    ccall((:mjp_getPlugin, libmujoco), Ptr{mjpPlugin}, (Ptr{Cchar}, Ptr{Cint}), name, slot)
end
"""
Look up a plugin by the registered slot number that was returned by mjp_registerPlugin.
"""
function mjp_getPluginAtSlot(slot)
    ccall((:mjp_getPluginAtSlot, libmujoco), Ptr{mjpPlugin}, (Cint,), slot)
end
"""
Set default resource provider definition.
"""
function mjp_defaultResourceProvider(provider)
    ccall((:mjp_defaultResourceProvider, libmujoco), Cvoid, (Ptr{mjpResourceProvider},), provider)
end
"""
Globally register a resource provider in a thread-safe manner. The provider must have a prefix that is not a sub-prefix or super-prefix of any current registered providers.  This function returns a slot number > 0 on success.
"""
function mjp_registerResourceProvider(provider)
    ccall((:mjp_registerResourceProvider, libmujoco), Cint, (Ptr{mjpResourceProvider},), provider)
end
"""
Return the number of globally registered resource providers.
"""
function mjp_resourceProviderCount()
    ccall((:mjp_resourceProviderCount, libmujoco), Cint, ())
end
"""
Return the resource provider with the prefix that matches against the resource name. If no match, return NULL.
"""
function mjp_getResourceProvider(resource_name)
    ccall((:mjp_getResourceProvider, libmujoco), Ptr{mjpResourceProvider}, (Ptr{Cchar},), resource_name)
end
"""
Look up a resource provider by slot number returned by mjp_registerResourceProvider. If invalid slot number, return NULL.
"""
function mjp_getResourceProviderAtSlot(slot)
    ccall((:mjp_getResourceProviderAtSlot, libmujoco), Ptr{mjpResourceProvider}, (Cint,), slot)
end
