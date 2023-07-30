using Clang.Generators
using MuJoCo_jll

cd(@__DIR__)

include_dir = normpath(MuJoCo_jll.artifact_dir, "include")
mujoco_dir = joinpath(include_dir, "mujoco")

# Wrapper generator options
options = load_options(joinpath(@__DIR__, "generator.toml"))

# add compiler flags, e.g. "-DXXXXXXXXX"
args = get_default_args()  # Must call this function first and then append your own flags
push!(args, "-I$include_dir")

headers = [joinpath(mujoco_dir, header) for header in readdir(mujoco_dir) if endswith(header, ".h")]

# create context
ctx = create_context(headers, args, options)


if !isdir("./LibMuJoCo")
    mkdir("./LibMuJoCo")
end

# run generator
build!(ctx)

if !isdir("../src/LibMuJoCo")
    mkdir("../src/LibMuJoCo")
end

# Fix the autogenerated code
lines_to_remove = [
    "const MJAPI",
    "const MJLOCAL",
    "const MJOPTION_FLOATS",
    "const MJOPTION_SCALARS",
    "const MJOPTION_VECTORS",
    "const XMJV",
    "const MJMODEL_INTS",
    "const MJMODEL_POINTERS",
    "const MJDATA_POINTERS",
    "const MJDATA_ARENA_POINTERS_CONTACT",
    "const MJDATA_ARENA_POINTERS_PRIMAL",
    "const MJDATA_ARENA_POINTERS_DUAL",
    "const MJDATA_ARENA_POINTERS",
    "const MJDATA_SCALAR",
    "const MJDATA_VECTOR",
    "const MJOPTION_INTS"
]

replace_math_functions = Dict{String, String}(
    "const mju_atan2 = atan2" => "const mju_atan2 = atan",
    "const mju_pow = pow" => "const mju_pow = ^",
    "const mju_abs = fabs" => "const mju_abs = abs",
)

file_lines = readlines("./LibMuJoCo/LibMuJoCo.jl")

function remove_lines_starting_with(lines, line_starting)
    filter(lines) do line
        return !any(startswith(line, s) for s in line_starting)
    end
end
function replace_lines_with(lines, replacements)
    c = 0
    new_lines = map(lines) do line
        if haskey(replacements, line)
            c += 1
            return replacements[line]
        else
            return line
        end
    end
    @assert length(replacements) == c
    return new_lines
end

multiline_remove = [
    "const mjMARKSTACK",
    "const mjFREESTACK"
]



file_lines = remove_lines_starting_with(file_lines, lines_to_remove)
file_lines = replace_lines_with(file_lines, replace_math_functions)

function remove_multilines(lines, multiline_removals)
    io = IOBuffer()
    is_removing = false

    for line in lines
        if any(startswith(line, m) for m in multiline_removals)
            is_removing = true
        end

        if is_removing
            if endswith(line, "end)))")
                is_removing = false
            end
        else
            println(io, line)
        end
    end
    return String(take!(io))
end

all_lines = remove_multilines(file_lines, multiline_remove);


open("./LibMuJoCo/LibMuJoCo.jl", "w") do io
    print(io, all_lines)
end


cp("./LibMuJoCo/LibMuJoCo.jl", "../src/LibMuJoCo/LibMuJoCo.jl"; force=true)
