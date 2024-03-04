using TestItemRunner

@testitem "Load XML" begin
    @test load_model(MuJoCo.humanoid_model_file()) isa MuJoCo.Model
end

@testitem "Load Broken XML" begin
    path = joinpath(@__DIR__, "configs", "broken_config.xml")
    @test_throws Exception load_model(path)
end

@testitem "Load binary MuJoCo" begin
    path = joinpath(@__DIR__, "configs", "binary_cartpole.mjb")
    @test load_model(path) isa MuJoCo.Model
end

@testitem "Initialise Data" begin
    model = load_model(MuJoCo.humanoid_model_file())

    @test init_data(model) isa MuJoCo.Data
end

@testitem "Step Simulation" begin
    model, data = MuJoCo.sample_model_and_data()
    step!(model, data)
end