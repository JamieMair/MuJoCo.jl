module Utils

export alloc

function alloc(::Type{T}) where {T}
    return Ptr{T}(Libc.malloc(sizeof(T)))
end

end