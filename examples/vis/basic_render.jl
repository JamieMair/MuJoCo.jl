cd(@__DIR__)
using Pkg
Pkg.activate("..")

using GLFW
using MuJoCo
using MuJoCo: LibMuJoCo

function alloc(::Type{T}) where {T}
    return Ptr{T}(Libc.malloc(sizeof(T)))
end

function testrender(xml="../../models/humanoid.xml")
    model = load_xml(xml)
    data = init_data(model)

    cam_ptr = alloc(LibMuJoCo.mjvCamera)
    opt_ptr = alloc(LibMuJoCo.mjvOption)
    scn_ptr = alloc(LibMuJoCo.mjvScene)
    con_ptr = alloc(LibMuJoCo.mjrContext)

    window = GLFW.CreateWindow(1920, 1080, "MuJoCo.jl")
    GLFW.MakeContextCurrent(window)
    GLFW.SwapInterval(1);

    LibMuJoCo.mjv_defaultCamera(cam_ptr)
    LibMuJoCo.mjv_defaultOption(opt_ptr)
    LibMuJoCo.mjv_defaultScene(scn_ptr)
    LibMuJoCo.mjr_defaultContext(con_ptr)

    LibMuJoCo.mjv_makeScene(model.internal_pointer, scn_ptr, 2000)
    LibMuJoCo.mjr_makeContext(model.internal_pointer, con_ptr, LibMuJoCo.mjFONTSCALE_150)

    while !GLFW.WindowShouldClose(window)

        # Render here
        num_steps = Int(ceil((1.0/60.0) / model.opt.timestep))
        println(num_steps)
        for _ in 1:num_steps
            step!(model, data)
        end
        
        viewport = LibMuJoCo.mjrRect(0, 0, 1920, 1080)
        
        LibMuJoCo.mjv_updateScene(model.internal_pointer, data.internal_pointer, opt_ptr, C_NULL, cam_ptr, LibMuJoCo.mjCAT_ALL, scn_ptr)

        LibMuJoCo.mjr_render(viewport, scn_ptr, con_ptr)

        # Swap front and back buffers
        GLFW.SwapBuffers(window)
    
        # Poll for and process events
        GLFW.PollEvents()
    end

    LibMuJoCo.mjv_freeScene(scn_ptr)
    LibMuJoCo.mjr_freeContext(con_ptr)

    LibMuJoCo.mj_deleteData(data.internal_pointer)
    LibMuJoCo.mj_deleteModel(model.internal_pointer)

    Libc.free(cam_ptr)
    Libc.free(opt_ptr)
    Libc.free(scn_ptr)
    Libc.free(con_ptr)

    GLFW.DestroyWindow(window)

    nothing
end

testrender()