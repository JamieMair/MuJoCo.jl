using MuJoCo
using Documenter

DocMeta.setdocmeta!(MuJoCo, :DocTestSetup, :(using MuJoCo); recursive=true)

makedocs(;
    modules=[MuJoCo],
    authors="Jamie Mair <JamieMair@users.noreply.github.com>, Nicholas Barbara <nic-barbara@users.noreply.github.com>",
    sitename="MuJoCo.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://JamieMair.github.io/MuJoCo.jl",
        repolink="https://github.com/JamieMair/MuJoCo.jl/blob/{commit}{path}#{line}",
        edit_link="main",
        size_threshold=typemax(Int),
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Introduction" => Any[
            "Getting Started" => "intro/getting_started.md",
            "Package Overview" => "intro/overview.md",
            "Contributing to the Package" => "intro/developing.md"
        ],
        "Examples" => Any[
            "Balancing a Cart-Pole" => "examples/cartpole_balance.md",
            "Humanoid LQR" => "examples/humanoid_lqr.md"
        ],
        "API" => "library/api.md",
        "C Bindings" => Any[
            "API" => "library/libmujoco.md",
            "Index" => "library/libmujoco_index.md"
        ]
    ],
    warnonly=true
)

deploydocs(;
    repo="github.com/JamieMair/MuJoCo.jl",
    devbranch="main",
)
