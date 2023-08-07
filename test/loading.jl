using TestItemRunner

@testitem "Load XML" begin
    @test load_xml(MuJoCo.sample_xml_filepath()) isa MuJoCo.Model
end

@testitem "Initialise Data" begin
    model = load_xml(MuJoCo.sample_xml_filepath())

    @test init_data(model) isa MuJoCo.Data
end

@testitem "Step Simulation" begin
    model, data = MuJoCo.sample_model_and_data()
    step!(model, data)
end