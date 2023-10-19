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
    fn_body = replace(fn_body,
        pointer_access_regex => s"\1.\2",
        pointer_defref => s"\1[\2+1]", # Convert to 1-based
        variable_assignment => s"\2 =",
        "}" => "end",
        "{" => ""
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
            is_const = isnothing(m.captures[1]),
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
            is_const = isnothing(m.captures[1]),
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
            is_optional = isnothing(m.captures[1]),
            is_inner_const = isnothing(m.captures[2]),
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
            is_const = isnothing(m.captures[1]),
            is_optional = isnothing(m.captures[2]),
            is_inner_const = isnothing(m.captures[3]),
            identifier = m.captures[5],
            datatype = LibMuJoCo.mjtNum,
        )
    end
    !isnothing(vector_info) && return vector_info

    # Matches vectors of various types (e.g. std::optional<Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>>> index)
    anomalous_vector_regex = r"(std::optional<)?Eigen::Ref<(const )?\s*Eigen::Vector<\s*\b([a-zA-Z_][a-zA-Z0-9_:]*)\s*,\s*\b(Eigen::Dynamic|[a-zA-Z0-9_]*)\s*>>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    anomalous_vector_info = if_match(anomalous_vector_regex, argument) do m
        is_dynamic = m.captures[4] == "Eigen::Dynamic"
        return (;
            type = :anomalous_vector,
            is_optional = isnothing(m.captures[1]),
            is_inner_const = isnothing(m.captures[2]),
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
    write(buffer, "\t@warn column_major_warning_string(\"$variable_name\")")
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
        write(buffer, "\terror(\"$variable_name should be a vector of size $expected_length\")")
        write(buffer, "end\n")
    end

    vector_type_check = "typeof($variable_name) <: AbstractArray{$datatype, 2} && count(==(1), size($variable_name)) < 1"
    if is_optional
        write(buffer, "if !isnothing($variable_name) && $vector_type_check\n")
    else
        write(buffer, "if $vector_type_check\n")
    end
    if !isnothing(expected_length)
        write(buffer, "\terror(\"$variable_name should be a vector of size $expected_length.\")")
    else
        write(buffer, "\terror(\"$variable_name should be a vector, not a matrix.\")")
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
map_to_abstract_supertype(::Type{<:AbstractFloat}) = AbstractFloat
map_to_abstract_supertype(::Type{<:UInt8}) = UInt8
map_to_abstract_supertype(x) = nothing

function basic_arg_transform(identifer, type)
    mapped_type = map_to_abstract_supertype(type)
    if !isnothing(mapped_type)
        return "$identifer::$mapped_type"
    else
        return identifer
    end
end

function convert_argument_from_info(info, fn_body, pre_body_buffer::IOBuffer, post_body_buffer::IOBuffer)
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
    post_body_buffer = IOBuffer()
    for arg_info in arg_infos
        arg = convert_argument_from_info(arg_info, fn_body, pre_body_buffer, post_body_buffer)
        push!(args, arg)
    end
    
    write(pre_body_buffer, fn_body)
    write(pre_body_buffer, String(take!(post_body_buffer)))

    # Overwrite function with new fn
    fn_body = String(take!(pre_body_buffer))

    return args, fn_body
end

function make_fn(fn_name, block)
    if fn_name in ("mj_saveLastXML", "mj_printSchema", "mj_saveModel", "mj_setLengthRange", "mju_printMatSparse", "mjv_updateScene", "mju_standardNormal")
        return nothing
    end


    anon_fn = extract_anon_fn(block)
    if any(x->!isspace(x), anon_fn)

        args, fn_body = extract_fn_info(anon_fn)

        fn_buffer = IOBuffer()
        write(fn_buffer, "function LibMuJoCo.$fn_name(")
        write(fn_buffer, join(args, ", "))
        write(fn_buffer, ")\n")
        write(fn_buffer, fn_body)
        if endswith(fn_body, "\n")
            write(fn_buffer, "end")
        else
            write(fn_buffer, "\nend")
        end

        return String(take!(fn_buffer))
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

    fn = make_fn(fn_name, block)
    return fn
end
function extract_other_def(block)
    trait_def = r"DEF_WITH_OMITTED_PY_ARGS\((traits)?::\b([a-zA-Z_][a-zA-Z0-9_]*)"
    m = match(trait_def, block)
    if isnothing(m)
        error("Could not find function name for block:\n$block")
    end
    fn_name = m.captures[2]

    fn = make_fn(fn_name, block)
    return fn
end



function extract_blocks(io, fileloc)
    fw = open(fileloc, "w")
    println(fw, "import LinearAlgebra")

    # Add a helper function for the warning
    println(fw, raw"""
    function column_major_warning_string(variable_name)
        return "$variable_name is stored in column-major order (Julia default), but mujoco expects arrays in row-major order. Use helper functions to generate row-major arrays and see documentation for more details."
    end
    """)
    

    while !eof(io)
        line = readline(io)
        if startswith(strip(line), "//") # skip comments
            continue
        end
        if contains(line, "Def<traits")
            fn_def = extract_normal_def(extract_text_block(io, line))
            !isnothing(fn_def) && println(fw, fn_def)
        elseif contains(line, "DEF_WITH_OMITTED_PY_ARGS")
            fn_def = extract_other_def(extract_text_block(io, line))
            !isnothing(fn_def) && println(fw, fn_def)
        end
    end
    close(fw)
    nothing
end


function write_function_constraints(staging_dir)
    file_location = joinpath(@__DIR__, "..", "mujoco", "python", "mujoco", "functions.cc")
    io = IOBuffer(join(readlines(file_location), "\n"))
    extract_blocks(io, joinpath(staging_dir, "function_constraints.jl"))
    @info "Finished writing function wrappers to check array sizes."
    # TODO: Make sure &(*point)[0] is handled correctly 
    nothing
end