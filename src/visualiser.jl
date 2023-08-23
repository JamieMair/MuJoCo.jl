# Creates definitions for the visualiser structs
module Visualiser

import ..LibMuJoCo
import ..LibMuJoCo: mjrContext, mjvOption, mjvCamera, mjvFigure, mjvScene, mjvPerturb
import ..LibMuJoCo: RendererContext, VisualiserOption, VisualiserCamera, VisualiserFigure, VisualiserScene, VisualiserPerturb
import ..Utils: alloc

function LibMuJoCo.RendererContext()
    mj_context = alloc(mjrContext)
    LibMuJoCo.mjr_defaultContext(mj_context)
    return RendererContext(mj_context)
end
function LibMuJoCo.VisualiserOption()
    mj_option = alloc(mjvOption)
    LibMuJoCo.mjv_defaultOption(mj_option)
    return VisualiserOption(mj_option)
end
function LibMuJoCo.VisualiserCamera()
    mj_camera = alloc(mjvCamera)
    LibMuJoCo.mjv_defaultCamera(mj_camera)
    return VisualiserCamera(mj_camera)
end
function LibMuJoCo.VisualiserFigure()
    mj_figure = alloc(mjvFigure)
    LibMuJoCo.mjv_defaultFigure(mj_figure)
    return VisualiserFigure(mj_figure)
end
function LibMuJoCo.VisualiserScene()
    mj_scene = alloc(mjvScene)
    LibMuJoCo.mjv_defaultScene(mj_scene)
    return VisualiserScene(mj_scene)
end
function LibMuJoCo.VisualiserPerturb()
    mj_perturb = alloc(mjvPerturb)
    LibMuJoCo.mjv_defaultPerturb(mj_perturb)
    return VisualiserPerturb(mj_perturb)
end

function test_visualiser end

function visualise end

export RendererContext, VisualiserOption, VisualiserCamera, VisualiserFigure, VisualiserScene, VisualiserPerturb, test_visualiser, visualise

end


