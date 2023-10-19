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

function convert_argument(argument)
    argument = strip(argument)

    # Static Arrays (e.g. const mjtNum (*axis)[3])
    static_array_regex = r"(const )?\s*\b([a-zA-Z_][a-zA-Z0-9_]*)\s*\(\*\s*\b([a-zA-Z_][a-zA-Z0-9_]*)\s*\)\s*\[\s*(\d+)\s*\]"
    static_array_info = if_match(static_array_regex, argument) do m
        return (;
            is_const = isnothing(m.captures[1]),
            array_type = m.captures[2],
            identifier = m.captures[3],
            array_size = m.captures[4]
        )
    end
    if !isnothing(static_array_info)
        return static_array_info.identifier
    end

    # Pointer to either MjModel or MjData
    const_pointer_regex = r"(const )?\s*(\w+::)?(MjModel|MjData)\*\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    const_pointer_info = if_match(const_pointer_regex, argument) do m
        return (;
            is_const = isnothing(m.captures[1]),
            namespace_prefix = m.captures[2],
            datatype = m.captures[3],
            identifier = m.captures[4]
        )
    end
    if !isnothing(const_pointer_info)
        return const_pointer_info.identifier
    end

    # Matches matrices (i.e. std::optional<Eigen::Ref<EigenArrayXX>> jacr)
    two_dim_array_regex = r"(std::optional<)?Eigen::Ref<(const )?\s*EigenArrayXX>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    two_dim_array_info = if_match(two_dim_array_regex, argument) do m
        return (;
            is_optional = isnothing(m.captures[1]),
            is_inner_const = isnothing(m.captures[2]),
            identifier = m.captures[4],
            datatype = LibMuJoCo.mjtNum,
        )
    end
    if !isnothing(two_dim_array_info)
        return two_dim_array_info.identifier
    end

    # Matches vectors (i.e. Eigen::Ref<const EigenVectorX> vec)
    vector_regex = r"(const )?\s*(std::optional<)?Eigen::Ref<(const )?\s*EigenVectorX>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    vector_info = if_match(vector_regex, argument) do m
        return (;
            is_const = isnothing(m.captures[1]),
            is_optional = isnothing(m.captures[2]),
            is_inner_const = isnothing(m.captures[3]),
            identifier = m.captures[5],
            datatype = LibMuJoCo.mjtNum,
        )
    end
    if !isnothing(vector_info)
        return vector_info.identifier
    end

    # Matches vectors of various types (e.g. std::optional<Eigen::Ref<Eigen::Vector<int, Eigen::Dynamic>>> index)
    anomalous_vector_regex = r"(std::optional<)?Eigen::Ref<(const )?\s*Eigen::Vector<\s*\b([a-zA-Z_][a-zA-Z0-9_:]*)\s*,\s*\b(Eigen::Dynamic|[a-zA-Z0-9_]*)\s*>>(>)?\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    anomalous_vector_info = if_match(anomalous_vector_regex, argument) do m
        return (;
            is_optional = isnothing(m.captures[1]),
            is_inner_const = isnothing(m.captures[2]),
            datatype = m.captures[3],
            size_vector = m.captures[4],
            identifier = m.captures[6],
        )
    end
    if !isnothing(anomalous_vector_info)
        return anomalous_vector_info.identifier
    end

    string_regex = r"(const )?\s*std::string&\s+\b([a-zA-Z_][a-zA-Z0-9_]*)"
    string_info = if_match(string_regex, argument) do m
        return (;
            is_const = isnothing(m.captures[1]),
            identifier = m.captures[2],
            datatype = String,
        )
    end
    if !isnothing(string_info)
        return string_info.identifier
    end


    basic_argument_regex = r"(int|float|mjtByte|mjtNum) \b([a-zA-Z_][a-zA-Z0-9_]*)"
    basic_argument_info = if_match(basic_argument_regex, argument) do m
        return (;
            datatype = m.captures[1],
            identifier = m.captures[2],
        )
    end
    if !isnothing(basic_argument_info)
        return basic_argument_info.identifier
    else
        @error "Could not parse $argument"
    end

    # Should not reach this point
    return nothing
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
    args = convert_argument.(arguments)

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
        write(fn_buffer, "\nend")

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
end
function extract_other_def(block)
    trait_def = r"DEF_WITH_OMITTED_PY_ARGS\((traits)?::\b([a-zA-Z_][a-zA-Z0-9_]*)"
    m = match(trait_def, block)
    if isnothing(m)
        error("Could not find function name for block:\n$block")
    end
    fn_name = m.captures[2]

    fn = make_fn(fn_name, block)
end



function extract_blocks(io, fileloc)
    fw = open(fileloc, "w")
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