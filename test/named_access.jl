using TestItemRunner

@testitem "Build a named model and data pair" begin

    model, data = MuJoCo.sample_model_and_data()
    using MuJoCo.NamedAccess

    named_model = NamedModel(model)
    named_data = NamedData(data, named_model)
end

@testitem "Can access all available named items for NamedModel" begin
    model, data = MuJoCo.sample_model_and_data()
    using MuJoCo.NamedAccess
    named_model = NamedModel(model)
    mapping = NamedAccess._raw_name_mapping(named_model)
    for (collection_type, name_map) in mapping
        fn = eval(:(MuJoCo.NamedAccess.$(collection_type)))
        for (n, i) in name_map
            @test fn(named_model, n) == fn(named_model, i)
            wrapper = fn(named_model, n)
            # test accessing all properties
            for pn in Base.propertynames(wrapper)
                @test typeof(getproperty(wrapper, pn)) <: Any
            end
        end
    end
end

@testitem "Can access all available named items for raw model" begin
    model, data = MuJoCo.sample_model_and_data()
    using MuJoCo.NamedAccess
    named_model = NamedModel(model)
    mapping = NamedAccess._raw_name_mapping(named_model)

    for (collection_type, name_map) in mapping
        fn = eval(:(MuJoCo.NamedAccess.$(collection_type)))
        for (n, i) in name_map
            @test fn(model, string(n)) == fn(model, i)
            wrapper = fn(model, string(n))
            # test accessing all properties
            for pn in Base.propertynames(wrapper)
                @test typeof(getproperty(wrapper, pn)) <: Any
            end
        end
    end
end

@testitem "Can access all available named items for NamedData" begin
    model, data = MuJoCo.sample_model_and_data()
    step!(model, data)

    using MuJoCo.NamedAccess
    named_model = NamedModel(model)
    named_data = NamedData(data, named_model)
    
    mapping = NamedAccess._raw_name_mapping(named_model)

    data_fn_names = (:actuator, :tendon, :site, :sensor, :body, :camera, :joint, :light, :geom) 
    for collection_type in data_fn_names
        fn = eval(:(MuJoCo.NamedAccess.$(collection_type)))
        if !haskey(mapping, collection_type)
            continue
        end
        name_map = mapping[collection_type]

        for (n, i) in name_map
            @test fn(named_data, n) == fn(named_data, i)
            wrapper = fn(named_data, n)
            # test accessing all properties
            for pn in Base.propertynames(wrapper)
                @test typeof(getproperty(wrapper, pn)) <: Any
            end
        end
    end
end


@testitem "Can access all available named items for raw data" begin
    model, data = MuJoCo.sample_model_and_data()
    step!(model, data)
    using MuJoCo.NamedAccess
    named_model = NamedModel(model)    
    mapping = NamedAccess._raw_name_mapping(named_model)

    data_fn_names = (:actuator, :tendon, :site, :sensor, :body, :camera, :joint, :light, :geom) 
    for collection_type in data_fn_names
        fn = eval(:(MuJoCo.NamedAccess.$(collection_type)))
        if !haskey(mapping, collection_type)
            continue
        end
        name_map = mapping[collection_type]

        for (n, i) in name_map
            @test fn(data, string(n)) == fn(data, i)
            wrapper = fn(data, string(n))
            # test accessing all properties
            for pn in Base.propertynames(wrapper)
                @test typeof(getproperty(wrapper, pn)) <: Any
            end
        end
    end
end


@testitem "Can access all types of collections for both model and data" begin
    model, data = MuJoCo.sample_model_and_data()
    step!(model, data)
    import MuJoCo as MJ
    
    fn_names = (:actuators, :tendons, :sites, :sensors, :bodies, :cameras, :joints, :lights, :geoms) 
    for fn_name in fn_names
        fn = getfield(MJ, fn_name)
        @test typeof(fn(model)) <: Tuple
        @test typeof(fn(data)) <: Tuple

        for x in fn(model)
            @test hasproperty(x, :name)
            @test typeof(x.name) <: Union{Symbol, AbstractString}
        end
        for x in fn(data)
            @test hasproperty(x, :name)
            @test typeof(x.name) <: Union{Symbol, AbstractString}
        end
    end    
end