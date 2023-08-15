module MuJoCo
using UnsafeArrays

include("LibMuJoCo/LibMuJoCo.jl")
include("utils.jl")
include("visualiser.jl")

using .LibMuJoCo
import .LibMuJoCo: Model, Data
export load_xml, init_data, step!
export init_visualiser, install_visualiser


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
    return Data(data_ptr, model) # Requires a reference to the model to get array sizes
end
function step!(model::Model, data::Data)
    mj_step(model.internal_pointer, data.internal_pointer)
end

function sample_model_and_data()
    model = load_xml(sample_xml_filepath())
    data = init_data(model)
    return model, data
end

# Handle backwards compatibility

if !isdefined(Base, :get_extension)
    using Requires
end

"""
    init_visualiser()

Loads the packages necessary for the running the visualiser.

Add the following packages to your project to be able to use
the visualisation features, or run "install_visualiser()".

# Packages:
- GLFW
- Observables
- StaticArrays
"""
function init_visualiser()
    @eval Main using GLFW, Observables, StaticArrays
    @eval Main Base.retry_load_extensions()
end
"""
    install_visualiser()

Installs the necessary packages for the running the visualiser
into the current running environment.

# Packages:
- GLFW
- Observables
- StaticArrays
"""
function install_visualiser()
    @eval Main import Pkg
    @eval Main Pkg.add(["GLFW", "Observables", "StaticArrays"])
end

@static if !isdefined(Base, :get_extension)
function __init__()
    @static if !isdefined(Base, :get_extension)
        @require GLFW="f7f18e0c-5ee9-5ccd-a5bf-e8befd85ed98" begin
            @require Observables = "510215fc-4207-5dde-b226-833fc4488ee2" begin
                @require StaticArrays = "90137ffa-7385-5640-81b9-e52037218182" begin
                    include("../ext/VisualiserExt/VisualiserExt.jl")
                end
            end
         end
    end
end
end


end