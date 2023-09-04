using MuJoCo
using Documenter

DocMeta.setdocmeta!(MuJoCo, :DocTestSetup, :(using MuJoCo); recursive=true)

makedocs(;
    modules=[MuJoCo],
    authors="Jamie Mair <JamieMair@users.noreply.github.com>, Nicholas Barbara <nic-barbara@users.noreply.github.com>",
    repo="https://github.com/JamieMair/MuJoCo.jl/blob/{commit}{path}#{line}",
    sitename="MuJoCo.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://JamieMair.github.io/MuJoCo.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "API" => "api.md",
        "Index" => "api_index.md",
        "C Bindings" => Any[
            "API" => "libmujoco.md",
            "Index" => "libmujoco_index.md"
        ]
    ],
)

deploydocs(;
    repo="github.com/JamieMair/MuJoCo.jl",
    devbranch="main",
)
