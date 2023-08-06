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
    
    @test data.nstack isa Integer
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