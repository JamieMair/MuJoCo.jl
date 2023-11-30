function index_by_name(model::Model, obj_type, name::String)
    return LibMuJoCo.mj_name2id(model, obj_type, name)
end
function index_by_name(data::Data, obj_type, name)
    model = getfield(data, :model)
    return index_by_name(model, obj_type, name)
end
function name_by_index(model::Model, obj_type, index)
    name_ptr = LibMuJoCo.mj_id2name(model, obj_type, index)
    if name_ptr == C_NULL
        return nothing
    else
        return unsafe_string(name_ptr)
    end    
end
function name_by_index(data::Data, obj_type, index)
    model = getfield(data, :model)
    return name_by_index(model, obj_type, index)
end


fieldoffset(T, x) = Base.fieldoffset(T, x)
function fieldoffset(T::Type, fname::Symbol)
    idx = findfirst(x->x==fname, fieldnames(T))
    if !isnothing(idx)
        return Base.fieldoffset(T, idx)
    else
        error("Unrecognised field $fname for $T")
    end
end


struct NamedModel
    model::Model
    name::Symbol
    name_to_index_mappings::Dict{Symbol, Dict{Symbol, Int}}
end
function _raw_name_mapping(x::NamedModel)
    return getfield(x, :name_to_index_mappings)
end
function NamedModel(model::Model)
    name_fieldnames = filter(fieldnames(LibMuJoCo.mjModel)) do fn
        fn_str = string(fn)
        startswith(fn_str, "name_") && endswith(fn_str, "adr")
    end

    names_char_ptr = (model.names').pointer
    
    name_to_index_mappings = Dict{Symbol, Dict{Symbol, Int}}()
    for fn in name_fieldnames
        identifier = Symbol(string(fn)[6:end-3])

        array_offsets = getproperty(model, fn)
        if !isnothing(array_offsets) && length(array_offsets) > 0
            m = Dict{Symbol, Int}()
            for (i, offset) in enumerate(array_offsets)
                matching_name = unsafe_string(Ptr{UInt8}(names_char_ptr + offset))
                if length(matching_name) > 0 && isvalid(matching_name)
                    m[Symbol(matching_name)] = i - 1 # 0 based indexing
                end
            end
            if length(m) > 0
                name_to_index_mappings[identifier] = m
            end
        end
    end
    model_name = Symbol(unsafe_string(Ptr{UInt8}(names_char_ptr)))

    return NamedModel(model, model_name, name_to_index_mappings)
end

struct NamedData
    data::Data
    model::NamedModel
end

function index_by_name(model::NamedModel, identifier::Symbol, name::Symbol)
    mappings = getfield(model, :name_to_index_mappings)
    return mappings[identifier][name]
end
function index_by_name(data::NamedData, identifier::Symbol, name::Symbol)
    return index_by_name(getfield(data, :model), identifier, name)
end
function name_by_index(model::NamedModel, obj_type, index)
    return name_by_index(getfield(model, :model), obj_type, index)
end
function name_by_index(data::NamedData, obj_type, index)
    return name_by_index(getfield(data, :data), obj_type, index)
end


# Overload base functions
Base.getproperty(x::NamedModel, f::Symbol) = getproperty(getfield(x, :model), f)
Base.getproperty(x::NamedData, f::Symbol) = getproperty(getfield(x, :data), f)
Base.propertynames(x::NamedModel) = propertynames(getfield(x, :model))
Base.propertynames(x::NamedData) = propertynames(getfield(x, :data))
Base.setproperty!(x::NamedModel, f::Symbol, v) = setproperty!(getfield(x, :model), f, v)
Base.setproperty!(x::NamedData, f::Symbol, v) = setproperty!(getfield(x, :data), f, v)

function Base.cconvert(::Type{Ptr{LibMuJoCo.mjModel}}, wrapper::NamedModel)
    Base.cconvert(Ptr{LibMuJoCo.mjModel}, getfield(wrapper, :model))
end
function Base.cconvert(::Type{Ptr{LibMuJoCo.mjData}}, wrapper::NamedData)
    Base.cconvert(Ptr{LibMuJoCo.mjData}, getfield(wrapper, :data))
end

export NamedModel, NamedData