module VisualiserExt

isdefined(Base, :get_extension) ? (import MuJoCo) : (import ..MuJoCo)
isdefined(Base, :get_extension) ? (import MuJoCo.LibMuJoCo) : (import ..MuJoCo.LibMuJoCo)
isdefined(Base, :get_extension) ? (import GLFW) : (import ..GLFW)



end