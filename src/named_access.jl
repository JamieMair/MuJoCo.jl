struct NamedModel{BN, JN}
    model::Model
    body_names::Val{BN}
    joint_names::Val{JN}
end

Base.propertynames(m::NamedModel) = Base.propertynames(m.model)
Base.getproperty(m::NamedModel, x) = Base.getproperty(m.model, x)
Base.setproperty!(m::NamedModel, x, v) = Base.getproperty(m.model, x, v)
Base.cconvert(::Type{Ptr{mjModel}}, wrapper::NamedModel) = Base.cconvert(Ptr{mjModel}, wrapper.model)

function _cstrptr_to_symbol(str_ptr::Ptr{Int32}, offset = 0)
    mem_address = unsafe_load(str_ptr)
    Symbol(unsafe_string(Ptr{UInt8}(mem_address + offset)))
end
function _cstrptr_to_symbol(str_ptr, offset = 0)
    Symbol(unsafe_string(Ptr{UInt8}(str_ptr + offset)))
end


function NamedModel(model::Model)
    names_char_arr = transpose(model.names)
    initial_offset = Int(names_char_arr.pointer)
    
    body_names = Tuple(_cstrptr_to_symbol(p, initial_offset) for p in model.name_bodyadr)
    joint_names = Tuple(_cstrptr_to_symbol(p, initial_offset) for p in model.name_jntadr)
    return NamedModel(model, Val(body_names), Val(joint_names))
end

function bodynames(::NamedModel{BN, JN}) where {BN, JN}
    return BN
end
function jointnames(::NamedModel{BN, JN}) where {BN, JN}
    return JN
end
function bodyindex(::NamedModel{BN, JN}, bodyname::Symbol) where {BN, JN}
    return findfirst(x->x==bodyname, BN)
end
function jointindex(::NamedModel{BN, JN}, jointname::Symbol) where {BN, JN}
    return findfirst(x->x==jointname, JN)
end
export NamedModel, bodynames, jointnames, bodyindex, jointindex