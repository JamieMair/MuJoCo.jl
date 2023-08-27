using UnsafeArrays
export RendererContext, VisualiserOption, VisualiserCamera, VisualiserFigure, VisualiserScene
mutable struct RendererContext
    internal_pointer::Ptr{mjrContext}
    function RendererContext(internal_pointer::Ptr{mjrContext})
        __renderercontext = new(internal_pointer)
        function __finalizer(__renderercontext)
            mjr_freeContext(__renderercontext.internal_pointer)
        end
        Base.finalizer(__finalizer, __renderercontext)
        return __renderercontext
    end
end
mutable struct VisualiserOption
    internal_pointer::Ptr{mjvOption}
    function VisualiserOption(internal_pointer::Ptr{mjvOption})
        __visualiseroption = new(internal_pointer)
        function __finalizer(__visualiseroption)
            Libc.free(__visualiseroption.internal_pointer)
        end
        Base.finalizer(__finalizer, __visualiseroption)
        return __visualiseroption
    end
end
mutable struct VisualiserCamera
    internal_pointer::Ptr{mjvCamera}
    function VisualiserCamera(internal_pointer::Ptr{mjvCamera})
        __visualisercamera = new(internal_pointer)
        function __finalizer(__visualisercamera)
            Libc.free(__visualisercamera.internal_pointer)
        end
        Base.finalizer(__finalizer, __visualisercamera)
        return __visualisercamera
    end
end
mutable struct VisualiserFigure
    internal_pointer::Ptr{mjvFigure}
    function VisualiserFigure(internal_pointer::Ptr{mjvFigure})
        __visualiserfigure = new(internal_pointer)
        function __finalizer(__visualiserfigure)
            Libc.free(__visualiserfigure.internal_pointer)
        end
        Base.finalizer(__finalizer, __visualiserfigure)
        return __visualiserfigure
    end
end
mutable struct VisualiserScene
    internal_pointer::Ptr{mjvScene}
    function VisualiserScene(internal_pointer::Ptr{mjvScene})
        __visualiserscene = new(internal_pointer)
        function __finalizer(__visualiserscene)
            mjv_freeScene(__visualiserscene.internal_pointer)
        end
        Base.finalizer(__finalizer, __visualiserscene)
        return __visualiserscene
    end
end
function Base.propertynames(x::RendererContext)
    (:lineWidth, :shadowClip, :shadowScale, :fogStart, :fogEnd, :fogRGBA, :shadowSize, :offWidth, :offHeight, :offSamples, :fontScale, :auxWidth, :auxHeight, :auxSamples, :offFBO, :offFBO_r, :offColor, :offColor_r, :offDepthStencil, :offDepthStencil_r, :shadowFBO, :shadowTex, :auxFBO, :auxFBO_r, :auxColor, :auxColor_r, :ntexture, :textureType, :texture, :basePlane, :baseMesh, :baseHField, :baseBuiltin, :baseFontNormal, :baseFontShadow, :baseFontBig, :rangePlane, :rangeMesh, :rangeHField, :rangeBuiltin, :rangeFont, :nskin, :skinvertVBO, :skinnormalVBO, :skintexcoordVBO, :skinfaceVBO, :charWidth, :charWidthBig, :charHeight, :charHeightBig, :glInitialized, :windowAvailable, :windowSamples, :windowStereo, :windowDoublebuffer, :currentBuffer, :readPixelFormat)
end
function Base.getproperty(x::RendererContext, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :lineWidth && return unsafe_load(Ptr{Float32}(internal_pointer + 0))
    f === :shadowClip && return unsafe_load(Ptr{Float32}(internal_pointer + 4))
    f === :shadowScale && return unsafe_load(Ptr{Float32}(internal_pointer + 8))
    f === :fogStart && return unsafe_load(Ptr{Float32}(internal_pointer + 12))
    f === :fogEnd && return unsafe_load(Ptr{Float32}(internal_pointer + 16))
    f === :fogRGBA && return UnsafeArray(Ptr{Float32}(internal_pointer + 20), (4,))
    f === :shadowSize && return unsafe_load(Ptr{Int32}(internal_pointer + 36))
    f === :offWidth && return unsafe_load(Ptr{Int32}(internal_pointer + 40))
    f === :offHeight && return unsafe_load(Ptr{Int32}(internal_pointer + 44))
    f === :offSamples && return unsafe_load(Ptr{Int32}(internal_pointer + 48))
    f === :fontScale && return unsafe_load(Ptr{Int32}(internal_pointer + 52))
    f === :auxWidth && return UnsafeArray(Ptr{Int32}(internal_pointer + 56), (10,))
    f === :auxHeight && return UnsafeArray(Ptr{Int32}(internal_pointer + 96), (10,))
    f === :auxSamples && return UnsafeArray(Ptr{Int32}(internal_pointer + 136), (10,))
    f === :offFBO && return unsafe_load(Ptr{UInt32}(internal_pointer + 176))
    f === :offFBO_r && return unsafe_load(Ptr{UInt32}(internal_pointer + 180))
    f === :offColor && return unsafe_load(Ptr{UInt32}(internal_pointer + 184))
    f === :offColor_r && return unsafe_load(Ptr{UInt32}(internal_pointer + 188))
    f === :offDepthStencil && return unsafe_load(Ptr{UInt32}(internal_pointer + 192))
    f === :offDepthStencil_r && return unsafe_load(Ptr{UInt32}(internal_pointer + 196))
    f === :shadowFBO && return unsafe_load(Ptr{UInt32}(internal_pointer + 200))
    f === :shadowTex && return unsafe_load(Ptr{UInt32}(internal_pointer + 204))
    f === :auxFBO && return UnsafeArray(Ptr{UInt32}(internal_pointer + 208), (10,))
    f === :auxFBO_r && return UnsafeArray(Ptr{UInt32}(internal_pointer + 248), (10,))
    f === :auxColor && return UnsafeArray(Ptr{UInt32}(internal_pointer + 288), (10,))
    f === :auxColor_r && return UnsafeArray(Ptr{UInt32}(internal_pointer + 328), (10,))
    f === :ntexture && return unsafe_load(Ptr{Int32}(internal_pointer + 368))
    f === :textureType && return UnsafeArray(Ptr{Int32}(internal_pointer + 372), (100,))
    f === :texture && return UnsafeArray(Ptr{UInt32}(internal_pointer + 772), (100,))
    f === :basePlane && return unsafe_load(Ptr{UInt32}(internal_pointer + 1172))
    f === :baseMesh && return unsafe_load(Ptr{UInt32}(internal_pointer + 1176))
    f === :baseHField && return unsafe_load(Ptr{UInt32}(internal_pointer + 1180))
    f === :baseBuiltin && return unsafe_load(Ptr{UInt32}(internal_pointer + 1184))
    f === :baseFontNormal && return unsafe_load(Ptr{UInt32}(internal_pointer + 1188))
    f === :baseFontShadow && return unsafe_load(Ptr{UInt32}(internal_pointer + 1192))
    f === :baseFontBig && return unsafe_load(Ptr{UInt32}(internal_pointer + 1196))
    f === :rangePlane && return unsafe_load(Ptr{Int32}(internal_pointer + 1200))
    f === :rangeMesh && return unsafe_load(Ptr{Int32}(internal_pointer + 1204))
    f === :rangeHField && return unsafe_load(Ptr{Int32}(internal_pointer + 1208))
    f === :rangeBuiltin && return unsafe_load(Ptr{Int32}(internal_pointer + 1212))
    f === :rangeFont && return unsafe_load(Ptr{Int32}(internal_pointer + 1216))
    f === :nskin && return unsafe_load(Ptr{Int32}(internal_pointer + 1220))
    f === :skinvertVBO && return unsafe_load(Ptr{Ptr{UInt32}}(internal_pointer + 1224))
    f === :skinnormalVBO && return unsafe_load(Ptr{Ptr{UInt32}}(internal_pointer + 1232))
    f === :skintexcoordVBO && return unsafe_load(Ptr{Ptr{UInt32}}(internal_pointer + 1240))
    f === :skinfaceVBO && return unsafe_load(Ptr{Ptr{UInt32}}(internal_pointer + 1248))
    f === :charWidth && return UnsafeArray(Ptr{Int32}(internal_pointer + 1256), (127,))
    f === :charWidthBig && return UnsafeArray(Ptr{Int32}(internal_pointer + 1764), (127,))
    f === :charHeight && return unsafe_load(Ptr{Int32}(internal_pointer + 2272))
    f === :charHeightBig && return unsafe_load(Ptr{Int32}(internal_pointer + 2276))
    f === :glInitialized && return unsafe_load(Ptr{Int32}(internal_pointer + 2280))
    f === :windowAvailable && return unsafe_load(Ptr{Int32}(internal_pointer + 2284))
    f === :windowSamples && return unsafe_load(Ptr{Int32}(internal_pointer + 2288))
    f === :windowStereo && return unsafe_load(Ptr{Int32}(internal_pointer + 2292))
    f === :windowDoublebuffer && return unsafe_load(Ptr{Int32}(internal_pointer + 2296))
    f === :currentBuffer && return unsafe_load(Ptr{Int32}(internal_pointer + 2300))
    f === :readPixelFormat && return unsafe_load(Ptr{Int32}(internal_pointer + 2304))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::RendererContext, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :lineWidth
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :shadowClip
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :shadowScale
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :fogStart
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 12), cvalue)
        return cvalue
    end
    if f === :fogEnd
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 16), cvalue)
        return cvalue
    end
    if f === :shadowSize
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 20), cvalue)
        return cvalue
    end
    if f === :offWidth
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 24), cvalue)
        return cvalue
    end
    if f === :offHeight
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 28), cvalue)
        return cvalue
    end
    if f === :offSamples
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 32), cvalue)
        return cvalue
    end
    if f === :fontScale
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 36), cvalue)
        return cvalue
    end
    if f === :offFBO
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 40), cvalue)
        return cvalue
    end
    if f === :offFBO_r
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 44), cvalue)
        return cvalue
    end
    if f === :offColor
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 48), cvalue)
        return cvalue
    end
    if f === :offColor_r
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 52), cvalue)
        return cvalue
    end
    if f === :offDepthStencil
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 56), cvalue)
        return cvalue
    end
    if f === :offDepthStencil_r
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 60), cvalue)
        return cvalue
    end
    if f === :shadowFBO
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 64), cvalue)
        return cvalue
    end
    if f === :shadowTex
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 68), cvalue)
        return cvalue
    end
    if f === :ntexture
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 72), cvalue)
        return cvalue
    end
    if f === :basePlane
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 76), cvalue)
        return cvalue
    end
    if f === :baseMesh
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 80), cvalue)
        return cvalue
    end
    if f === :baseHField
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 84), cvalue)
        return cvalue
    end
    if f === :baseBuiltin
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 88), cvalue)
        return cvalue
    end
    if f === :baseFontNormal
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 92), cvalue)
        return cvalue
    end
    if f === :baseFontShadow
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 96), cvalue)
        return cvalue
    end
    if f === :baseFontBig
        cvalue = convert(UInt32, value)
        unsafe_store!(Ptr{UInt32}(internal_pointer + 100), cvalue)
        return cvalue
    end
    if f === :rangePlane
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 104), cvalue)
        return cvalue
    end
    if f === :rangeMesh
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 108), cvalue)
        return cvalue
    end
    if f === :rangeHField
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 112), cvalue)
        return cvalue
    end
    if f === :rangeBuiltin
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 116), cvalue)
        return cvalue
    end
    if f === :rangeFont
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 120), cvalue)
        return cvalue
    end
    if f === :nskin
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 124), cvalue)
        return cvalue
    end
    if f === :charHeight
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 128), cvalue)
        return cvalue
    end
    if f === :charHeightBig
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 132), cvalue)
        return cvalue
    end
    if f === :glInitialized
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 136), cvalue)
        return cvalue
    end
    if f === :windowAvailable
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 140), cvalue)
        return cvalue
    end
    if f === :windowSamples
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 144), cvalue)
        return cvalue
    end
    if f === :windowStereo
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 148), cvalue)
        return cvalue
    end
    if f === :windowDoublebuffer
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 152), cvalue)
        return cvalue
    end
    if f === :currentBuffer
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 156), cvalue)
        return cvalue
    end
    if f === :readPixelFormat
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 160), cvalue)
        return cvalue
    end
    if f in (:fogRGBA, :auxWidth, :auxHeight, :auxSamples, :auxFBO, :auxFBO_r, :auxColor, :auxColor_r, :textureType, :texture, :charWidth, :charWidthBig)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    if f in (:skinvertVBO, :skinnormalVBO, :skintexcoordVBO, :skinfaceVBO)
        error("Cannot overwrite a pointer field.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjrContext}}, wrapper::RendererContext)
    return wrapper.internal_pointer
end
function Base.propertynames(x::VisualiserOption)
    (:label, :frame, :geomgroup, :sitegroup, :jointgroup, :tendongroup, :actuatorgroup, :skingroup, :flags, :bvh_depth)
end
function Base.getproperty(x::VisualiserOption, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :label && return unsafe_load(Ptr{Int32}(internal_pointer + 0))
    f === :frame && return unsafe_load(Ptr{Int32}(internal_pointer + 4))
    f === :geomgroup && return UnsafeArray(Ptr{UInt8}(internal_pointer + 8), (6,))
    f === :sitegroup && return UnsafeArray(Ptr{UInt8}(internal_pointer + 14), (6,))
    f === :jointgroup && return UnsafeArray(Ptr{UInt8}(internal_pointer + 20), (6,))
    f === :tendongroup && return UnsafeArray(Ptr{UInt8}(internal_pointer + 26), (6,))
    f === :actuatorgroup && return UnsafeArray(Ptr{UInt8}(internal_pointer + 32), (6,))
    f === :skingroup && return UnsafeArray(Ptr{UInt8}(internal_pointer + 38), (6,))
    f === :flags && return UnsafeArray(Ptr{UInt8}(internal_pointer + 44), (25,))
    f === :bvh_depth && return unsafe_load(Ptr{Int32}(internal_pointer + 72))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::VisualiserOption, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :label
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :frame
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :bvh_depth
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f in (:geomgroup, :sitegroup, :jointgroup, :tendongroup, :actuatorgroup, :skingroup, :flags)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjvOption}}, wrapper::VisualiserOption)
    return wrapper.internal_pointer
end
function Base.propertynames(x::VisualiserCamera)
    (:type, :fixedcamid, :trackbodyid, :lookat, :distance, :azimuth, :elevation)
end
function Base.getproperty(x::VisualiserCamera, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :type && return unsafe_load(Ptr{Int32}(internal_pointer + 0))
    f === :fixedcamid && return unsafe_load(Ptr{Int32}(internal_pointer + 4))
    f === :trackbodyid && return unsafe_load(Ptr{Int32}(internal_pointer + 8))
    f === :lookat && return UnsafeArray(Ptr{Float64}(internal_pointer + 16), (3,))
    f === :distance && return unsafe_load(Ptr{Float64}(internal_pointer + 40))
    f === :azimuth && return unsafe_load(Ptr{Float64}(internal_pointer + 48))
    f === :elevation && return unsafe_load(Ptr{Float64}(internal_pointer + 56))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::VisualiserCamera, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :type
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :fixedcamid
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :trackbodyid
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :distance
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 12), cvalue)
        return cvalue
    end
    if f === :azimuth
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 20), cvalue)
        return cvalue
    end
    if f === :elevation
        cvalue = convert(Float64, value)
        unsafe_store!(Ptr{Float64}(internal_pointer + 28), cvalue)
        return cvalue
    end
    if f in (:lookat,)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjvCamera}}, wrapper::VisualiserCamera)
    return wrapper.internal_pointer
end
function Base.propertynames(x::VisualiserFigure)
    (:flg_legend, :flg_ticklabel, :flg_extend, :flg_barplot, :flg_selection, :flg_symmetric, :linewidth, :gridwidth, :gridsize, :gridrgb, :figurergba, :panergba, :legendrgba, :textrgb, :linergb, :range, :xformat, :yformat, :minwidth, :title, :xlabel, :linename, :legendoffset, :subplot, :highlight, :highlightid, :selection, :linepnt, :linedata, :xaxispixel, :yaxispixel, :xaxisdata, :yaxisdata)
end
function Base.getproperty(x::VisualiserFigure, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :flg_legend && return unsafe_load(Ptr{Int32}(internal_pointer + 0))
    f === :flg_ticklabel && return UnsafeArray(Ptr{Int32}(internal_pointer + 4), (2,))
    f === :flg_extend && return unsafe_load(Ptr{Int32}(internal_pointer + 12))
    f === :flg_barplot && return unsafe_load(Ptr{Int32}(internal_pointer + 16))
    f === :flg_selection && return unsafe_load(Ptr{Int32}(internal_pointer + 20))
    f === :flg_symmetric && return unsafe_load(Ptr{Int32}(internal_pointer + 24))
    f === :linewidth && return unsafe_load(Ptr{Float32}(internal_pointer + 28))
    f === :gridwidth && return unsafe_load(Ptr{Float32}(internal_pointer + 32))
    f === :gridsize && return UnsafeArray(Ptr{Int32}(internal_pointer + 36), (2,))
    f === :gridrgb && return UnsafeArray(Ptr{Float32}(internal_pointer + 44), (3,))
    f === :figurergba && return UnsafeArray(Ptr{Float32}(internal_pointer + 56), (4,))
    f === :panergba && return UnsafeArray(Ptr{Float32}(internal_pointer + 72), (4,))
    f === :legendrgba && return UnsafeArray(Ptr{Float32}(internal_pointer + 88), (4,))
    f === :textrgb && return UnsafeArray(Ptr{Float32}(internal_pointer + 104), (3,))
    f === :linergb && return UnsafeArray(Ptr{Float32}(internal_pointer + 116), (100, 3))
    f === :range && return UnsafeArray(Ptr{Float32}(internal_pointer + 1316), (2, 2))
    f === :xformat && return UnsafeArray(Ptr{Int8}(internal_pointer + 1332), (20,))
    f === :yformat && return UnsafeArray(Ptr{Int8}(internal_pointer + 1352), (20,))
    f === :minwidth && return UnsafeArray(Ptr{Int8}(internal_pointer + 1372), (20,))
    f === :title && return UnsafeArray(Ptr{Int8}(internal_pointer + 1392), (1000,))
    f === :xlabel && return UnsafeArray(Ptr{Int8}(internal_pointer + 2392), (100,))
    f === :linename && return UnsafeArray(Ptr{Int8}(internal_pointer + 2492), (100, 100))
    f === :legendoffset && return unsafe_load(Ptr{Int32}(internal_pointer + 12492))
    f === :subplot && return unsafe_load(Ptr{Int32}(internal_pointer + 12496))
    f === :highlight && return UnsafeArray(Ptr{Int32}(internal_pointer + 12500), (2,))
    f === :highlightid && return unsafe_load(Ptr{Int32}(internal_pointer + 12508))
    f === :selection && return unsafe_load(Ptr{Float32}(internal_pointer + 12512))
    f === :linepnt && return UnsafeArray(Ptr{Int32}(internal_pointer + 12516), (100,))
    f === :linedata && return UnsafeArray(Ptr{Float32}(internal_pointer + 12916), (100, 2000))
    f === :xaxispixel && return UnsafeArray(Ptr{Int32}(internal_pointer + 812916), (2,))
    f === :yaxispixel && return UnsafeArray(Ptr{Int32}(internal_pointer + 812924), (2,))
    f === :xaxisdata && return UnsafeArray(Ptr{Float32}(internal_pointer + 812932), (2,))
    f === :yaxisdata && return UnsafeArray(Ptr{Float32}(internal_pointer + 812940), (2,))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::VisualiserFigure, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :flg_legend
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :flg_extend
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :flg_barplot
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :flg_selection
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 12), cvalue)
        return cvalue
    end
    if f === :flg_symmetric
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 16), cvalue)
        return cvalue
    end
    if f === :linewidth
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 20), cvalue)
        return cvalue
    end
    if f === :gridwidth
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 24), cvalue)
        return cvalue
    end
    if f === :legendoffset
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 28), cvalue)
        return cvalue
    end
    if f === :subplot
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 32), cvalue)
        return cvalue
    end
    if f === :highlightid
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 36), cvalue)
        return cvalue
    end
    if f === :selection
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 40), cvalue)
        return cvalue
    end
    if f in (:flg_ticklabel, :gridsize, :gridrgb, :figurergba, :panergba, :legendrgba, :textrgb, :linergb, :range, :xformat, :yformat, :minwidth, :title, :xlabel, :linename, :highlight, :linepnt, :linedata, :xaxispixel, :yaxispixel, :xaxisdata, :yaxisdata)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjvFigure}}, wrapper::VisualiserFigure)
    return wrapper.internal_pointer
end
function Base.propertynames(x::VisualiserScene)
    (:maxgeom, :ngeom, :geoms, :geomorder, :nskin, :skinfacenum, :skinvertadr, :skinvertnum, :skinvert, :skinnormal, :nlight, :lights, :camera, :enabletransform, :translate, :rotate, :scale, :stereo, :flags, :framewidth, :framergb)
end
function Base.getproperty(x::VisualiserScene, f::Symbol)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && return internal_pointer
    f === :maxgeom && return unsafe_load(Ptr{Int32}(internal_pointer + 0))
    f === :ngeom && return unsafe_load(Ptr{Int32}(internal_pointer + 4))
    f === :geoms && return unsafe_load(Ptr{Ptr{mjvGeom_}}(internal_pointer + 8))
    f === :geomorder && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 16))
    f === :nskin && return unsafe_load(Ptr{Int32}(internal_pointer + 24))
    f === :skinfacenum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 32))
    f === :skinvertadr && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 40))
    f === :skinvertnum && return unsafe_load(Ptr{Ptr{Int32}}(internal_pointer + 48))
    f === :skinvert && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 56))
    f === :skinnormal && return unsafe_load(Ptr{Ptr{Float32}}(internal_pointer + 64))
    f === :nlight && return unsafe_load(Ptr{Int32}(internal_pointer + 72))
    f === :lights && return UnsafeArray(Ptr{mjvLight_}(internal_pointer + 76), (100,))
    f === :camera && return UnsafeArray(Ptr{mjvGLCamera_}(internal_pointer + 8476), (2,))
    f === :enabletransform && return unsafe_load(Ptr{UInt8}(internal_pointer + 8588))
    f === :translate && return UnsafeArray(Ptr{Float32}(internal_pointer + 8592), (3,))
    f === :rotate && return UnsafeArray(Ptr{Float32}(internal_pointer + 8604), (4,))
    f === :scale && return unsafe_load(Ptr{Float32}(internal_pointer + 8620))
    f === :stereo && return unsafe_load(Ptr{Int32}(internal_pointer + 8624))
    f === :flags && return UnsafeArray(Ptr{UInt8}(internal_pointer + 8628), (10,))
    f === :framewidth && return unsafe_load(Ptr{Int32}(internal_pointer + 8640))
    f === :framergb && return UnsafeArray(Ptr{Float32}(internal_pointer + 8644), (3,))
    error("Could not find property $(f)")
end
function Base.setproperty!(x::VisualiserScene, f::Symbol, value)
    internal_pointer = getfield(x, :internal_pointer)
    f === :internal_pointer && error("Cannot set the internal pointer, create a new struct instead.")
    if f === :maxgeom
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 0), cvalue)
        return cvalue
    end
    if f === :ngeom
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 4), cvalue)
        return cvalue
    end
    if f === :nskin
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 8), cvalue)
        return cvalue
    end
    if f === :nlight
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 12), cvalue)
        return cvalue
    end
    if f === :enabletransform
        cvalue = convert(UInt8, value)
        unsafe_store!(Ptr{UInt8}(internal_pointer + 16), cvalue)
        return cvalue
    end
    if f === :scale
        cvalue = convert(Float32, value)
        unsafe_store!(Ptr{Float32}(internal_pointer + 17), cvalue)
        return cvalue
    end
    if f === :stereo
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 21), cvalue)
        return cvalue
    end
    if f === :framewidth
        cvalue = convert(Int32, value)
        unsafe_store!(Ptr{Int32}(internal_pointer + 25), cvalue)
        return cvalue
    end
    if f in (:lights, :camera, :translate, :rotate, :flags, :framergb)
        error("Cannot overwrite array field. Mutate the array instead.")
    end
    if f in (:geoms, :geomorder, :skinfacenum, :skinvertadr, :skinvertnum, :skinvert, :skinnormal)
        error("Cannot overwrite a pointer field.")
    end
    error("Could not find property $(f) to set.")
end
function Base.cconvert(::Type{Ptr{mjvScene}}, wrapper::VisualiserScene)
    return wrapper.internal_pointer
end
