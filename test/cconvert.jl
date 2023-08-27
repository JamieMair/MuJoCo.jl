using TestItemRunner

@testitem "Accessing value internals" begin
    using MuJoCo.LibMuJoCo
    model, data = MuJoCo.sample_model_and_data()

    # Test whether model and data can be auto converted to the C pointer.
    @test Base.cconvert(Ptr{mjModel}, model) isa Ptr{mjModel}
    @test Base.cconvert(Ptr{mjData}, data) isa Ptr{mjData}
end