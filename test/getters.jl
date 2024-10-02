using TestItemRunner


@testitem "Accessing value internals" begin
    model, data = MuJoCo.sample_model_and_data()

    # Test whether model and data can access the internal pointer
    @test model.internal_pointer isa Ptr
    @test data.internal_pointer isa Ptr


    # Test whether values can be accessed as pure numbers
    @test model.nq isa Integer
    @test model.nv isa Integer
    @test model.nbody isa Integer
    @test model.nsensordata isa Integer
    
    @test data.narena isa Integer
    @test data.nbuffer isa Integer
end

@testitem "Accessing array properties" begin
    import MuJoCo.LibMuJoCo
    using UnsafeArrays
    _, data = MuJoCo.sample_model_and_data()

    for (fname, ftype) in zip(fieldnames(LibMuJoCo.mjData), fieldtypes(LibMuJoCo.mjData))
        if ftype <: NTuple
            a = getproperty(data, fname)
            @test typeof(a) <: AbstractArray
            size_a_bytes = sizeof(eltype(a)) * reduce(*, size(a))
            @test size_a_bytes == sizeof(ftype) # Make sure the array is the right size
            
            @test begin
                x = a[1] # Test accessing elements
                y = a[end]
                true
            end

            if hasmethod(zero, Tuple{eltype(a)})
                @test begin
                    a[1] = zero(eltype(a)) # Test setting elements
                    a[end] = zero(eltype(a)) # Test setting elements
                    true
                end
            end
        end
    end
end


@testitem "Accessing wrapped structs" begin
    using UnsafeArrays
    model, data = MuJoCo.sample_model_and_data()

    # Test whether wrapped structs are created
    @test model.opt isa MuJoCo.Options
    @test model.stat isa MuJoCo.Statistics

    # Test chained accessing
    @test model.opt.timestep isa AbstractFloat
    @test model.opt.gravity isa UnsafeArray
    @test length(model.opt.gravity) == 3
end

@testitem "Test array pointer alignment" begin
    using UnsafeArrays
    model, data = MuJoCo.sample_model_and_data()

    g = model.opt.gravity
    @test g isa UnsafeArray
    @test g ≈ [0.0, 0.0, -9.81]
end

@testitem "Test pointer arrays" begin
    using UnsafeArrays
    model, data = MuJoCo.sample_model_and_data()

    c = data.ctrl
    @test c isa AbstractArray
    @test size(c) == (21, 1)
end

@testitem "Test that all fields can be accessed without error." begin
    using UnsafeArrays
    model, data = MuJoCo.sample_model_and_data()

    @testset "Model accessors" begin
        for fname in Base.propertynames(model)
            @test typeof(getproperty(model, fname)) <: Any
        end
    end
    @testset "Data accessors" begin
        for fname in Base.propertynames(data)
            @test typeof(getproperty(data, fname)) <: Any
        end
    end
end

@testitem "Manually setting time variable in data struct" begin
    using MuJoCo
    model, data = MuJoCo.sample_model_and_data()
    data.time = 10.0
    @test data.time == 10.0
end