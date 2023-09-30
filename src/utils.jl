module Utils

export alloc

function alloc(::Type{T}) where {T}
    return Ptr{T}(Libc.malloc(sizeof(T)))
end

function show_array(io, x::AbstractArray)
    size_arr = size(x)
    if length(size_arr) == 1
        print(io, "($(length(x)) x  )")
    else
        print(io, '(')
        join(io, size_arr, " x ")
        print(io, ')')
    end
    print(io, "\t")
    show(io, eltype(x))
    print(io, "\t")
    print(io, '[')
    sigdigits = 4
    round_fn(x::AbstractFloat) = round(x; sigdigits)
    round_fn(x) = x
    join(io, Iterators.map(round_fn, x), ", ")
    print(io, ']')
    if eltype(x) <: AbstractFloat
        print(io, " ($sigdigits s.f.)")
    end
end

end