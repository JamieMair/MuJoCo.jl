function extract_text_block(io, starting_line)
    buffer = IOBuffer()
    write(buffer, starting_line)
    bracket_num = 0;
    line = starting_line
    should_stop = false
    while true
        for char in line
            if char == '('
                bracket_num += 1
            elseif char == ')'
                bracket_num -= 1
            elseif char == ';'
                if bracket_num == 0
                    should_stop = true
                    break # stop once at the end
                end
            end
        end

        if should_stop
            break
        end
        line = readline(io)
        write(buffer, '\n')
        write(buffer, line)
    end

    return strip(String(take!(buffer)))
end

function extract_anon_fn(block)
    fn_buffer = IOBuffer()
    bracket_num = 0
    start_collection = false
    has_started_fn = false
    curly_bracket_num = 0
    for char in block
        if char == '('
            bracket_num += 1
        elseif char == ')'
            bracket_num -= 1
        end

        if bracket_num > 0
            if char == ',' && !start_collection
                start_collection = true
            elseif start_collection && has_started_fn
                write(fn_buffer, char)
                if char == '{'
                    curly_bracket_num += 1
                elseif char == '}'
                    curly_bracket_num -= 1
                    if curly_bracket_num == 0
                        has_started_fn = false
                    end
                end
            elseif char == '['
                has_started_fn = true
                if fn_buffer.size > 0
                    error("Unexpected second function definition when parsing function constraints. Block:\n$block")
                end
                write(fn_buffer, char)
            end
        elseif bracket_num == 0 && start_collection
            start_collection = false # stop collecting when closing a bracket
        end
    end

    return String(take!(fn_buffer))
end

function convert_fn_body(fn_body)

    # Pre-clean
    index_offset_match = r"\[([^[]+?)\s?-\s?1\s?\]"
    double_string = Regex("\"([^\"]*)\"\\n\\s*\"(([^\"]*))\"")
    fn_body = replace(fn_body, 
        index_offset_match=>s"[\1]",
        double_string => s"\"\1 \2\""
    )

    # Added below to replace all dimension and size extractions
    rows_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)(?:.|->)rows\(\)"
    cols_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)(?:.|->)cols\(\)"
    size_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)(?:.|->)size\(\)"





    # Change to Julia syntax
    exception_regex = r"throw py::type_error\(\n?(.*)\);"
    data_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)(?:.|->)data\(\)"
    cstr_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)(?:.|->)c_str\(\)"
    intercept_error_regex = r"InterceptMjErrors\(::\b([a-zA-Z_][a-zA-Z0-9_]*)\)"
    return_fn_regex = r"return ::\b([a-zA-Z_][a-zA-Z0-9_]*)\("
    error_regex = r"throw FatalError\(std::string\((.*)\)\);"
    other_exception = r"throw \b([a-zA-Z_][a-zA-Z0-9_]*)\((.+)\);"
    has_value_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)(?:.|->)has_value\(\)"

    fn_body = replace(fn_body,
        rows_regex=>s"size(\1, 1)",
        cols_regex=>s"size(\1, 2)",
        size_regex=>s"length(\1)",
        exception_regex=>s"throw(ArgumentError(\1))",
        data_regex => s"\1",
        cstr_regex => s"\1",
        intercept_error_regex=> s"\1",
        return_fn_regex => s"return \1(",
        error_regex => s"error(String(\1))",
        other_exception => s"error(\1)",
        has_value_regex => s"!isnothing(\1)",
        "nullptr"=>"C_NULL"
    )


    

    # Modify pointer references
    pointer_access_regex = r"\b([a-zA-Z_][a-zA-Z0-9_]*)->\b([a-zA-Z_][a-zA-Z0-9_]*)"
    pointer_defref = r"&\(\*\b([a-zA-Z_][a-zA-Z0-9_]*)\)\[(\d+)\]"
    variable_assignment = r"\b([a-zA-Z_][a-zA-Z0-9_]*)\s+\b([a-zA-Z_][a-zA-Z0-9_]*)\s*="
    # Misc
    return_non_module_fn_regex = r"return \b([a-zA-Z_][a-zA-Z0-9_]*)\("
    fn_body = replace(fn_body,
        pointer_access_regex => s"\1.\2",
        pointer_defref => s"\1[\2+1]", # Convert to 1-based
        variable_assignment => s"\2 =",
        "}" => "end",
        "{" => "",
        return_non_module_fn_regex => s"return LibMuJoCo.\1(",
    )

    return fn_body
end

function if_match(fn, regex, text)
    m = match(regex, text)
    if !isnothing(m)
        return fn(m)
    end
    return nothing
end

function convert_datatype(dtype)
    dtype_conversion = Dict{String, DataType}(
        "int"=>Int32,
        "float"=>Float32,
        "double"=>Float64,
        "mjtNum"=>LibMuJoCo.mjtNum,
        "mjtByte"=>LibMuJoCo.mjtByte,
        "MjModel"=>LibMuJoCo.mjModel,
        "MjData"=>LibMuJoCo.mjData,
    )

    if !haskey(dtype_conversion, dtype)
        error("Datatype $dtype is not recognised")
    end

    return dtype_conversion[dtype] 
end
function convert_size(s)
    if !isnothing(tryparse(Int, s))
        return parse(Int, s)
    elseif (hasproperty(LibMuJoCo, Symbol(s)))
        return getproperty(LibMuJoCo, Symbol(s))
    else
        error("Unknown size $s")
        return s
    end
end

function extract_arg_info(argument)
    argument = strip(argument)

    # Static Arrays (e.g. const mjtNum (*axis)[3])
    static_array_regex = r"(const )?\s*\b([a-zA-Z_][a-zA-Z0-9_]*)\s*\(\*\s*\b([a-zA-Z_][a-zA-Z0-9_]*)\s*\)\s*\[\s*(\d+)\s*\]"
    static_array_info = if_match(static_array_regex, argument) do m
        return (;
            type = :static_array,
            is_const = !isnothing(m.captures[1]),
            datatype = convert_datatype(m.captures[2]),
            identifier = m.captures[3],
            array_size = convert_size(m.captures[4])
        )
    end
    !isnothing(static_array_info) && return static_array_info

    # Pointer to either MjModel or MjData
    const_pointer_regex = r"(const )?\s*(\w+::)?(MjModel|MjData)\*\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    const_pointer_info = if_match(const_pointer_regex, argument) do m
        is_model = m.captures[3] == "MjModel"
        return (;
            type = is_model ? (:mjModel) : (:mjData),
            is_const = !isnothing(m.captures[1]),
            namespace_prefix = m.captures[2],
            datatype = convert_datatype(m.captures[3]),
            identifier = m.captures[4]
        )
    end
    !isnothing(const_pointer_info) && return const_pointer_info

    # Matches matrices (i.e. std::optional<Eigen::Ref<EigenArrayXX>> jacr)
    two_dim_array_regex = r"(std::optional<)?Eigen::Ref<(const )?\s*EigenArrayXX>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    two_dim_array_info = if_match(two_dim_array_regex, argument) do m
        return (;
            type = :matrix,
            is_optional = !isnothing(m.captures[1]),
            is_inner_const = !isnothing(m.captures[2]),
            identifier = m.captures[4],
            datatype = LibMuJoCo.mjtNum,
        )
    end
    !isnothing(two_dim_array_info) && return two_dim_array_info

    # Matches vectors (i.e. Eigen::Ref<const EigenVectorX> vec)
    vector_regex = r"(const )?\s*(std::optional<)?Eigen::Ref<(const )?\s*EigenVectorX>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    vector_info = if_match(vector_regex, argument) do m
        return (;
            type = :variable_vector,
            is_const = !isnothing(m.captures[1]),
            is_optional = !isnothing(m.captures[2]),
            is_inner_const = !isnothing(m.captures[3]),
            identifier = m.captures[5],
            datatype = LibMuJoCo.mjtNum,
        )
    end
    !isnothing(vector_info) && return vector_info

    ivector_regex = r"(const )?\s*(std::optional<)?Eigen::Ref<(const )?\s*EigenVectorI>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    ivector_info = if_match(ivector_regex, argument) do m
        return (;
            type = :variable_vector,
            is_const = !isnothing(m.captures[1]),
            is_optional = !isnothing(m.captures[2]),
            is_inner_const = !isnothing(m.captures[3]),
            identifier = m.captures[5],
            datatype = convert_datatype("int"),
        )
    end
    !isnothing(ivector_info) && return ivector_info


    # Matches vectors of various types (e.g. std::optional<Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>>> index)
    anomalous_vector_regex = r"(std::optional<)?Eigen::Ref<(const )?\s*Eigen::Vector<\s*\b([a-zA-Z_][a-zA-Z0-9_:]*)\s*,\s*\b(Eigen::Dynamic|[a-zA-Z0-9_]*)\s*>>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    anomalous_vector_info = if_match(anomalous_vector_regex, argument) do m
        is_dynamic = m.captures[4] == "Eigen::Dynamic"
        return (;
            type = :anomalous_vector,
            is_optional = !isnothing(m.captures[1]),
            is_inner_const = !isnothing(m.captures[2]),
            datatype = convert_datatype(m.captures[3]),
            is_dynamic_size = is_dynamic,
            size_vector = is_dynamic ? m.captures[4] : convert_size(m.captures[4]),
            identifier = m.captures[6],
        )
    end
    !isnothing(anomalous_vector_info) && return anomalous_vector_info

    string_regex = r"(const )?\s*std::string&\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    string_info = if_match(string_regex, argument) do m
        return (;
            type = :string,
            is_const = isnothing(m.captures[1]),
            identifier = m.captures[2],
            datatype = String,
        )
    end
    !isnothing(string_info) && return string_info


    basic_argument_regex = r"(int|float|double|mjtByte|mjtNum) \b([a-zA-Z_][a-zA-Z0-9_]*)"
    basic_argument_info = if_match(basic_argument_regex, argument) do m
        return (;
            type = :basic,
            datatype = convert_datatype(m.captures[1]),
            identifier = m.captures[2],
        )
    end
    if !isnothing(basic_argument_info)
        return basic_argument_info
    else
        error("Could not parse $argument")
    end

    # Should not reach this point
    return nothing
end

function write_matrix_order_warning_check(buffer::IOBuffer, variable_name, datatype, is_optional)
    row_major_type = "LinearAlgebra.Transpose{$datatype, Matrix{$datatype}}"
    if is_optional
        write(buffer, "if !isnothing($variable_name) && !(typeof($variable_name) <: $row_major_type)\n")
    else
        write(buffer, "if !(typeof($variable_name) <: $row_major_type)\n")
    end
    write(buffer, "    @warn column_major_warning_string(\"$variable_name\")")
    write(buffer, "end\n")
end
function write_vector_size_check(buffer::IOBuffer, variable_name, datatype, expected_length, is_optional)
    if !isnothing(expected_length)
        size_check = "length($variable_name) != $(expected_length)"
        if is_optional
            write(buffer, "if !isnothing($variable_name) && $size_check\n")
        else
            write(buffer, "if $size_check\n")
        end
        write(buffer, "    throw(ArgumentError(\"$variable_name should be a vector of size $expected_length\"))")
        write(buffer, "end\n")
    end

    vector_type_check = "typeof($variable_name) <: AbstractArray{$datatype, 2} && count(==(1), size($variable_name)) < 1"
    if is_optional
        write(buffer, "if !isnothing($variable_name) && $vector_type_check\n")
    else
        write(buffer, "if $vector_type_check\n")
    end
    if !isnothing(expected_length)
        write(buffer, "    throw(ArgumentError(\"$variable_name should be a vector of size $expected_length.\"))")
    else
        write(buffer, "    throw(ArgumentError(\"$variable_name should be a vector, not a matrix.\"))")
    end
    write(buffer, "end\n")
end
function optional_wrapper(identifier, is_optional, other_types...)
    if is_optional
        joined_other_types = join(other_types, ", ")
        return "$identifier::Union{Nothing, $joined_other_types}"
    elseif length(other_types) > 1
        joined_other_types = join(other_types, ", ")
        return "$identifier::Union{$joined_other_types}"
    elseif length(other_types) == 1
        return "$identifier::$(other_types[begin])"
    else
        throw(ArgumentError("other_types must have at least one entry."))
    end
end

map_to_abstract_supertype(::Type{<:Integer}) = Integer
map_to_abstract_supertype(::Type{<:AbstractFloat}) = Real
map_to_abstract_supertype(::Type{<:UInt8}) = Union{UInt8, Bool}
map_to_abstract_supertype(x) = nothing

function basic_arg_transform(identifer, type)
    mapped_type = map_to_abstract_supertype(type)
    if !isnothing(mapped_type)
        return "$identifer::$mapped_type"
    else
        return identifer
    end
end

function convert_argument_from_info(info, pre_body_buffer::IOBuffer)
    info_types = (:basic, :string, :anomalous_vector, :variable_vector, :matrix, :mjModel, :mjData, :static_array)
    if !(info.type in info_types)
        error("Unrecognised info type $(info.type)")
    end

    info.type == :basic && return basic_arg_transform(info.identifier, info.datatype)
    info.type == :string && return "$(info.identifier)::String"
    if info.type == :matrix
        write_matrix_order_warning_check(pre_body_buffer, info.identifier, info.datatype, info.is_optional)
        return optional_wrapper(info.identifier, info.is_optional, "AbstractArray{$(info.datatype), 2}")
    end
    if info.type == :anomalous_vector
        allowed_types = ("AbstractVector{$(info.datatype)}", "AbstractArray{$(info.datatype), 2}")
        write_vector_size_check(pre_body_buffer, info.identifier, info.datatype, !info.is_dynamic_size ? info.size_vector : nothing, info.is_optional)

        return optional_wrapper(
            info.identifier,
            info.is_optional,
            allowed_types...            
        )
    end
    if info.type == :variable_vector
        allowed_types = ("AbstractVector{$(info.datatype)}", "AbstractArray{$(info.datatype), 2}")
        write_vector_size_check(pre_body_buffer, info.identifier, info.datatype, nothing, info.is_optional)

        return optional_wrapper(
            info.identifier,
            info.is_optional,
            allowed_types...            
        )
    end
    if info.type == :static_array
        allowed_types = ("AbstractVector{$(info.datatype)}", "AbstractArray{$(info.datatype), 2}")
        write_vector_size_check(pre_body_buffer, info.identifier, info.datatype, info.array_size, false)
        return optional_wrapper(
            info.identifier,
            false,
            allowed_types...            
        )
    end
    info.type == :mjModel && return info.identifier # not strictly typed
    info.type == :mjData && return info.identifier # not strictly typed
    # Fallback
    return info.identifier
end


function get_argument_constraints(fn_body, arg_infos)
    type_error = r"""throw\(ArgumentError\(\n?"(.*)"\)\)"""

    pure_constraints = unique(map(eachmatch(type_error, fn_body)) do m
        strip(m.captures[begin], Set(('.', ' ')))
    end)

    parameter_constraints = map(arg_infos) do i
        _regex = Regex("\\s$(i.identifier)[\\s.]")
        constraints = map(pure_constraints) do c
            c_with_space = " " * c
            m = match(_regex, c_with_space)
            isnothing(m) && return nothing
            return strip(replace(c_with_space, _regex => " `$(i.identifier)` "))
        end

        return (i.identifier) => [c for c in constraints if !isnothing(c)]
    end

    constraint_info = Dict(parameter_constraints...)

    return constraint_info
end

function argument_doc_description(arg_info, constraints)

    constrain_text = join(constraints, ". ")
    if length(constrain_text) > 0
        constrain_text = " $constrain_text." # Add a space to the front and full stop at end
    end

    function construct_argdoc(identifier, type, is_const = false, description = "", pre_const_extra = "")
        x = "- **`$(identifier)::$(type)`**"
        if is_const || length(constrain_text) > 0 || length(description) > 0 || length(pre_const_extra) > 0
            x *= " ->"
        end

        if length(description) > 0
            x *= " " * description
        end
        if length(constrain_text) > 0
            x *= constrain_text
        end
        if length(pre_const_extra) > 0
            x *= pre_const_extra
        end
        if is_const
            x *= " Constant."
        end

        return x
    end

    if arg_info.type == :basic
        return construct_argdoc(arg_info.identifier, arg_info.datatype)
    elseif arg_info.type == :string
        return construct_argdoc(arg_info.identifier, "String", arg_info.is_const)
    elseif arg_info.type == :mjModel
        return construct_argdoc(arg_info.identifier, "Model", arg_info.is_const)
    elseif arg_info.type == :mjData
        return construct_argdoc(arg_info.identifier, "Data", arg_info.is_const)
    elseif arg_info.type == :static_array
        desc = "A vector of size $(arg_info.array_size)."
        return construct_argdoc(arg_info.identifier, "Vector{$(arg_info.datatype)}", arg_info.is_const, desc)
    end
    
    opt = arg_info.is_optional ? "An **optional**" : "A"
    opt_end = arg_info.is_optional ? " Can set to `nothing` if not required." : ""

    if arg_info.type == :anomalous_vector
        desc = if arg_info.is_dynamic_size
            "$opt vector of variable size."
        else
            "$opt vector of size $(arg_info.size_vector)."
        end
        return construct_argdoc(arg_info.identifier, "Vector{$(arg_info.datatype)}", arg_info.is_inner_const, desc, opt_end)
    elseif arg_info.type == :variable_vector
        desc = "$opt vector of variable size."
        return construct_argdoc(arg_info.identifier, "Vector{$(arg_info.datatype)}", arg_info.is_inner_const, desc, opt_end)
    elseif arg_info.type == :matrix
        desc = "$opt matrix of variable size."
        return construct_argdoc(arg_info.identifier, "Matrix{$(arg_info.datatype)}", arg_info.is_inner_const, desc, opt_end)
    else
        error("Unrecognised argument info for variable $(arg_info)")
    end
end

function write_argument_doc_description!(io, argument_infos, constraint_infos)
    if length(argument_infos) == 0
        return
    end

    println(io, "# Arguments")
    for arg_info in argument_infos
        constraints = constraint_infos[arg_info.identifier]
        println(io, argument_doc_description(arg_info, constraints))
    end
    nothing
end


function create_wrapped_docstring(fn_name, fn_body, argument_infos)
    # original_fn = getproperty(LibMuJoCo, Symbol(fn_name))
    original_documentation = string(eval(:(@doc LibMuJoCo.$(Symbol(fn_name)))))
    
    io = IOBuffer()

    write(io, "\"\"\"\n")
    # Write function signature
    write(io, "    ")
    write(io, fn_name)
    write(io, "(")
    write(io, join(map(x->x.identifier, argument_infos), ", "))
    write(io, ")\n\n")

    for line in split(original_documentation, "\n")[5:end]
        println(io, line)
    end

    constraint_info = get_argument_constraints(fn_body, argument_infos)

    write_argument_doc_description!(io, argument_infos, constraint_info)
    
    write(io, "\n")

    write(io, "\"\"\"\n")

    return String(take!(io))
end

function extract_fn_info(fn_block)
    bracket_num = 0
    angle_bracket_count = 0
    curly_bracket_count = 0
    has_finished_args = false

    arguments = String[]
    arg_buffer = IOBuffer()

    fn_body_buffer = IOBuffer()

    function consume_arg()
        if arg_buffer.size > 0
            push!(arguments, String(take!(arg_buffer)))
        end
        nothing
    end
    

    for char in fn_block
        if char == '('
            bracket_num += 1
        elseif char == ')'
            bracket_num -= 1
            if bracket_num == 0 && !has_finished_args
                has_finished_args = true
                consume_arg()
                continue
            end
        end

        if !has_finished_args
            if char == '<'
                angle_bracket_count += 1
            elseif char == '>'
                angle_bracket_count -= 1
            elseif angle_bracket_count == 0 && char == ','
                consume_arg()
                continue
            end
            
            write(arg_buffer, char)
        else
            if char == '{'
                curly_bracket_count += 1
                if curly_bracket_count == 1
                    continue # go to next character                    
                end
            elseif char == '}'
                curly_bracket_count -= 1
                if curly_bracket_count == 0
                    continue # go to next character                
                end
            end

            write(fn_body_buffer, char)
        end
    end

    fn_body = convert_fn_body(String(take!(fn_body_buffer)))
    arg_infos = extract_arg_info.(arguments)

    args = String[]
    pre_body_buffer = IOBuffer()
    for arg_info in arg_infos
        arg = convert_argument_from_info(arg_info, pre_body_buffer)
        push!(args, arg)
    end
    
    write(pre_body_buffer, fn_body)

    # Overwrite function with new fn
    fn_body = String(take!(pre_body_buffer))

    return args, arg_infos, fn_body
end

function make_fn(fn_name, block)
    if fn_name in ("mj_saveLastXML", "mj_printSchema", "mj_saveModel", "mj_setLengthRange", "mju_printMatSparse", "mjv_updateScene", "mju_standardNormal", "mj_loadAllPluginLibraries")
        return nothing
    end


    anon_fn = extract_anon_fn(block)
    if any(x->!isspace(x), anon_fn)

        args, arg_infos, fn_body = extract_fn_info(anon_fn)

        fn_buffer = IOBuffer()

        write(fn_buffer, create_wrapped_docstring(fn_name, fn_body, arg_infos))

        write(fn_buffer, "function $fn_name(")
        write(fn_buffer, join(args, ", "))
        write(fn_buffer, ")\n")
        write(fn_buffer, fn_body)
        if endswith(fn_body, "\n")
            write(fn_buffer, "end")
        else
            write(fn_buffer, "\nend")
        end

        return fn_name, String(take!(fn_buffer))
    end

    return nothing
end

function extract_normal_def(block)
    trait_def = r"Def<traits::([^>]+)>"
    m = match(trait_def, block)
    if isnothing(m)
        error("Could not find function name for block:\n$block")
    end
    fn_name = m.captures[begin]

    return make_fn(fn_name, block)
end
function extract_other_def(block)
    trait_def = r"DEF_WITH_OMITTED_PY_ARGS\((traits)?::\b([a-zA-Z_][a-zA-Z0-9_]*)"
    m = match(trait_def, block)
    if isnothing(m)
        error("Could not find function name for block:\n$block")
    end
    fn_name = m.captures[2]

    return make_fn(fn_name, block)
end



function extract_blocks(io, fileloc)
    fw = open(fileloc, "w")
    print_warning(fw)

    println(fw, "import LinearAlgebra")
    println(fw, "import .LibMuJoCo")

    # Add a helper function for the warning
    println(fw, raw"""
    function column_major_warning_string(variable_name)
        return "$variable_name is stored in column-major order (Julia default), but mujoco expects arrays in row-major order. Use helper functions to generate row-major arrays and see documentation for more details."
    end
    """)
    
    fn_names = String[]
    fn_defs = String[]
    while !eof(io)
        line = readline(io)
        if startswith(strip(line), "//") # skip comments
            continue
        end
        if contains(line, "Def<traits")
            result = extract_normal_def(extract_text_block(io, line))
            if !isnothing(result)
                fn_name, fn_def = result
                push!(fn_names, fn_name)
                push!(fn_defs, fn_def)
            end
        elseif contains(line, "DEF_WITH_OMITTED_PY_ARGS")
            result = extract_other_def(extract_text_block(io, line))
            if !isnothing(result)
                fn_name, fn_def = result
                push!(fn_names, fn_name)
                push!(fn_defs, fn_def)
            end
        end
    end

    println(fw, "export " * join(fn_names, ", "))

    println(fw, "\n")

    for fn_def in fn_defs
        println(fw, fn_def)
    end

    println(fw, "\n# Tuple for exports")
    println(fw, "const _wrapped_fns = (" * join(Iterators.map(x->":$x", fn_names), ", ") * ")")
    close(fw)
    
    
    return (;
        exports_to_remove = fn_names
    )
end


function write_function_constraints(libmujoco_filepath, staging_dir)
    file_location = joinpath(@__DIR__, "..", "mujoco", "python", "mujoco", "functions.cc")
    io = IOBuffer(join(readlines(file_location), "\n"))
    info = extract_blocks(io, joinpath(staging_dir, "function_constraints.jl"))
    @info "Finished writing function wrappers to check array sizes."

    alter_libmujoco_exports(libmujoco_filepath, Symbol.(info.exports_to_remove))
    # TODO: Some constraints like mju_encodePyramid are not being extracted now
    # TODO: Make sure &(*point)[0] is handled correctly 
    nothing
end

function alter_libmujoco_exports(libmujoco_filepath, export_blacklist)
    @info "Rewriting LibMuJoCo.jl with an export blacklist"
    libmujoco_module_expr = Meta.parse(join(readlines(libmujoco_filepath), "\n"))
    block_expr = libmujoco_module_expr.args[3]

    new_block_args = map(block_expr.args) do arg
        if !(typeof(arg) <: Expr) || arg.head != :for
            return arg
        end

        return :(
            for name in names(@__MODULE__(); all=true)
                if name in export_blacklist
                    continue
                end
                name_str = string(name)
                if any(startswith(name_str, prefix) for prefix in PREFIXES)
                    @eval export $(Expr(:$, :name))
                end
            end
        )
    end 

    blacklist_expr = Expr(:const, Expr(:(=), :export_blacklist, Expr(:tuple, QuoteNode.(export_blacklist)...)))
    insert!(new_block_args, length(new_block_args)-1, blacklist_expr)

    new_module_expr = Expr(:module, true, :LibMuJoCo, Expr(:block, new_block_args...))
    create_file_from_expr(libmujoco_filepath * ".new", new_module_expr)
    nothing
end