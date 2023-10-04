using TestItemRunner

@testitem "Create a row-major array" begin
    using MuJoCo

    x_dim = 20
    y_dim = 10
    vector_data = rand(x_dim * y_dim)

    array = Array(reshape(vector_data, x_dim, y_dim))

    carray = mj_array(eltype(array), x_dim, y_dim)
    for i in 1:length(vector_data)
        carray[i] = vector_data[i]
    end

    @test all(array .== carray)
end
@testitem "Create a row-major array (with tuple)" begin
    using MuJoCo

    x_dim = 20
    y_dim = 10
    vector_data = rand(x_dim * y_dim)

    array = Array(reshape(vector_data, x_dim, y_dim))

    carray = mj_array(eltype(array), (x_dim, y_dim))
    for i in 1:length(vector_data)
        carray[i] = vector_data[i]
    end

    @test all(array .== carray)
end
@testitem "Create a row-major array (with zeros)" begin
    using MuJoCo

    x_dim = 20
    y_dim = 10

    carray_tuple = mj_zeros(Float64, (x_dim, y_dim))
    carray_normal = mj_zeros(Float64, x_dim, y_dim)
    @test all(carray_normal .== carray_tuple .== 0)
end
@testitem "Create a row-major array with different element types." begin
    using MuJoCo

    x_dim = 5
    y_dim = 4

    carray_bools = mj_zeros(Bool, (x_dim, y_dim))
    carray_int8s = mj_zeros(Int8, (x_dim, y_dim))
    carray_int16s = mj_zeros(Int16, (x_dim, y_dim))
    @test all(carray_bools .== carray_int8s .== carray_int8s .== carray_int16s)
end

@testitem "Test actual row-major indexing." begin
    using MuJoCo
    using MuJoCo.UnsafeArrays
    x_dim = 4
    y_dim = 5
    carray = mj_zeros(Float64, (x_dim, y_dim))
    for i in eachindex(carray)
        carray[i] = rand(Float64)
    end

    GC.@preserve carray begin
        cptr = Base.pointer(carray, 1)
        raw_array = UnsafeArray(cptr, (y_dim, x_dim))

        @test all(transpose(raw_array).==carray)
    end
end