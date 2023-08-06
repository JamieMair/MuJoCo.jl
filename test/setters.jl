using TestItemRunner

@testitem "Setting value internals." begin
    model, data = MuJoCo.sample_model_and_data()

    # Test whether internal values can be set
    @test begin
        model.nv = 0
        model.nv = 0.0
        model.nv = 1.0f0 # Test conversion
        true
    end
end

@testitem "Check errors when performing invalid settings." begin
    model, data = MuJoCo.sample_model_and_data()
    if VERSION > v"1.8"
        @test_throws "Cannot overwrite a pointer field." begin
            model.buffer = Ptr{Cvoid}(0)
        end
        @test_throws "Cannot overwrite array field. Mutate the array instead." begin
            data.energy = [0.0, 0.0]
        end
    else
        @test_throws ErrorException begin
            model.buffer = Ptr{Cvoid}(0)
        end
        @test_throws ErrorException begin
            data.energy = [0.0, 0.0]
        end
    end
end

@testitem "Check array broadcasting to unsafe array." begin
    model, data = MuJoCo.sample_model_and_data()
    @test begin # Can broadcast into an array
        data.energy .= [0.0, 0.0]
        true
    end
end

@testitem "Check nested setting" begin
    model, data = MuJoCo.sample_model_and_data()
    @test begin
        model.opt.timestep = 0.000001
        true
    end
end