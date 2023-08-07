using TestItemRunner

@testitem "Building controller" begin
    using Random
    model, data = MuJoCo.sample_model_and_data()
    step!(data, model)
    controller = Controller(model, data)
    for i in 1:20
        randn!(controller.control)
        controller.control .*= 0.01
        step!(data, model)
    end
end