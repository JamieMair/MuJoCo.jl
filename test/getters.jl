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