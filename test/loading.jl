using TestItemRunner

@testitem "Load XML" begin
    using MuJoCo
    xml = joinpath(@__DIR__, "..", "models", "cartpole.xml")
    @test load_xml(xml) isa MuJoCo.Model
end

@testitem "Initialise Data" begin
    using MuJoCo
    xml = joinpath(@__DIR__, "..", "models", "cartpole.xml")
    model = load_xml(xml)

    @test init_data(model) isa MuJoCo.Data
end

@testitem "Step Simulation" begin
    using MuJoCo
    xml = joinpath(@__DIR__, "..", "models", "cartpole.xml")
    model = load_xml(xml)

    data = init_data(model)

    step!(data, model)
end