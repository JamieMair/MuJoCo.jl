module MuJoCo

include("LibMuJoCo/LibMuJoCo.jl")
using .LibMuJoCo
import .LibMuJoCo: jlModel, jlData

export jlModel, jlData, load_xml, init_data, step!, sample_xml_filepath

function sample_xml_filepath()
    return abspath(joinpath(abspath(@__DIR__), "..", "models", "humanoid.xml"))
end

function load_xml(path)
    error_msg = "Could not load XML model from $path"
    model_ptr = mj_loadXML(path, Ptr{Cvoid}(), error_msg, length(error_msg))#
    return jlModel(model_ptr)
end
function init_data(model::jlModel)
    data_ptr = mj_makeData(model.internal_pointer)
    return jlData(data_ptr)
end
function step!(data::jlData, model::jlModel)
    mj_step(model.internal_pointer, data.internal_pointer)
end

end