module MuJoCo

using UnsafeArrays
include("LibMuJoCo/LibMuJoCo.jl")
using .LibMuJoCo
import .LibMuJoCo: Model, Data
export load_xml, init_data, step!

function sample_xml_filepath()
    return abspath(joinpath(abspath(@__DIR__), "..", "models", "humanoid.xml"))
end

function load_xml(path)
    error_msg = "Could not load XML model from $path"
    !isfile(path) && error(error_msg)
    model_ptr = mj_loadXML(path, Ptr{Cvoid}(), error_msg, length(error_msg))
    return Model(model_ptr)
end
function init_data(model::Model)
    data_ptr = mj_makeData(model.internal_pointer)
    return Data(data_ptr)
end
function step!(model::Model, data::Data)
    mj_step(model.internal_pointer, data.internal_pointer)
end

function sample_model_and_data()
    model = load_xml(sample_xml_filepath())
    data = init_data(model)
    return model, data
end

end