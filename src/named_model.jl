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
function NamedModel(model::Model)
    name_fieldnames = filter(fieldnames(mjModel)) do fn
        fn_str = string(fn)
        startswith(fn_str, "name_") && endswith(fn_str, "adr")
    end

    names_char_ptr = (model.names').pointer
    
    name_to_index_mappings = Dict{Symbol, Dict{Symbol, Int}}()
    for fn in name_fieldnames
        identifier = Symbol(string(fn)[6:end-3])

        array_offsets = getproperty(model, fn)
        if length(array_offsets) > 0
            m = Dict{Symbol, Int}()
            for (i, offset) in enumerate(array_offsets)
                matching_name = unsafe_string(Ptr{UInt8}(names_char_ptr + offset))
                if length(matching_name) > 0 && isvalid(matching_name)
                    m[Symbol(matching_name)] = i
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

# Overload base functions
Base.getproperty(x::NamedModel, f::Symbol) = getproperty(getfield(x, :model), f)
Base.getproperty(x::NamedData, f::Symbol) = getproperty(getfield(x, :data), f)
Base.propertynames(x::NamedModel) = propertynames(getfield(x, :model))
Base.propertynames(x::NamedData) = propertynames(getfield(x, :data))
Base.setproperty!(x::NamedModel, f::Symbol, v) = setproperty!(getfield(x, :model), f, v)
Base.setproperty!(x::NamedData, f::Symbol, v) = setproperty!(getfield(x, :data), f, v)
