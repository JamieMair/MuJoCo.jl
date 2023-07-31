using MuJoCo_jll
using Git
cd(@__DIR__)

# Clone the mujoco directory here
if !isdir("./mujoco")
    run(`$(git()) clone https://github.com/deepmind/mujoco.git`)
end

mj_version = "2.3.7"
cd("./mujoco")
run(`$(git()) checkout tags/$(mj_version)`)
cd("../")

include_dir = normpath(MuJoCo_jll.artifact_dir, "include")
mujoco_headers_dir = joinpath(include_dir, "mujoco")



