
export example_model_files_directory, example_model_files, sample_model_and_data, load_model


function example_model_files_directory()
    return abspath(joinpath(MuJoCo_jll.artifact_dir, "share", "mujoco", "model"))
end
function example_model_files()
    root_path = example_model_files_directory()
    modelfiles = filter(x->last(splitext(x))==".xml", readdir(root_path, join=true))
    return modelfiles
end
function humanoid_model_file()
    return joinpath(example_model_files_directory(), "humanoid.xml")
end

function model_type(path::AbstractString)
    ext = Symbol(lowercase(last(split(last(splitext(path)), "."))))
    if ext in (:xml, :mjcf)
        return :MJCF
    elseif ext == :mjb
        return :MJB
    else
        throw(ArgumentError("Unrecognised model file extension. Must use one of 'xml', 'mjcf', or 'mjb' (or uppercase variants).\n\tExtension: $ext\n\tModel Path: $path"))
    end
end
function load_model(path::AbstractString, type::Symbol)
    if !isfile(path)
        throw(ArgumentError("Supplied model path could not be found. Path: $path"))
    end

    !(type in (:MJCF, :MJB)) && error("The file type must be either MJCF or MJB.")
    mpointer = if type == :MJCF
        error_msg = "Could not load XML model from $path"
        LibMuJoCo.mj_loadXML(path, Ptr{Cvoid}(), error_msg, length(error_msg))
    elseif type == :MJB
        error_msg = "Could not load MJB model from $path"
        mpointer = LibMuJoCo.mj_loadModel(path, Ptr{Cvoid}())
        mpointer == C_NULL && error(error_msg)
        mpointer
    end

    return Model(mpointer)
end

function load_model(path::AbstractString)
    mtype = model_type(path)
    load_model(path, mtype)
end

function sample_model_and_data()
    model = load_model(humanoid_model_file())
    data = init_data(model)
    return model, data
end