
export example_model_files_directory, example_model_files, sample_model_and_data, load_model

"""
Returns the absolute path of the root directory containing all the example model files.
"""
function example_model_files_directory()
    return abspath(joinpath(MuJoCo_jll.artifact_dir, "share", "mujoco", "model"))
end
"""
Returns the list of absolute paths to available example model files provided by MuJoCo.
"""
function example_model_files()
    root_path = example_model_files_directory()
    modelfiles = filter(x->last(splitext(x))==".xml", readdir(root_path, join=true))
    return modelfiles
end
"""
Returns the absolute path to the humanoid model provided by MuJoCo.
"""
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

"""
    load_model(path)

Determines the type of file by the extension and loads the model into memory.

To use this model in a simulator, you will also need the corresponding data,
obtained using [`init_data`](@ref).

Expected files types: 'xml', 'mjcf', or 'mjb' (or uppercase variants).

# Examples

```julia
model = load_model(MuJoCo.humanoid_model_file())
data = init_data(model)
```
"""
function load_model(path::AbstractString)
    mtype = model_type(path)
    load_model(path, mtype)
end

"""
    sample_model_and_data()

A utility module to create and initialise example `Model` and `Data` objects,
reflecting the underlying `mjModel` and `mjData` structs to provide REPL code
completition to aid development.

Returns a (model, data) tuple.
"""
function sample_model_and_data()
    model = load_model(humanoid_model_file())
    data = init_data(model)
    return model, data
end