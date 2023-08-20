# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

# Anything commented out is a function we have copied but not yet changed. Some of these will not be required in our final version and can be deleted.

const STEPSPERKEY = 1
const SHIFTSTEPSPERKEY = 50

function default_mousemovecb(v::MuJoCoViewer, s::WindowState, ev::MouseMoveEvent)
    p = v.phys
    m = p.model
    d = p.data
    ui = v.ui

    if ev.isdrag
        # Move free camera

        scaled_dx = ev.dx / s.width
        scaled_dy = ev.dy / s.height

        if s.right
            action = s.shift ? LibMuJoCo.mjMOUSE_MOVE_H : LibMuJoCo.mjMOUSE_MOVE_V
        elseif s.left
            action = s.shift ? LibMuJoCo.mjMOUSE_ROTATE_H : LibMuJoCo.mjMOUSE_ROTATE_V
        else
            action = LibMuJoCo.mjMOUSE_ZOOM
        end

        if iszero(p.pert.active)
            mjv_moveCamera(m.internal_pointer, action, scaled_dx, scaled_dy, ui.scn.internal_pointer, ui.cam.internal_pointer)
        else
            mjv_movePerturb(m.internal_pointer, d.internal_pointer, action, scaled_dx, scaled_dy, ui.scn.internal_pointer, p.pert.internal_pointer)
        end
    end

    return
end

function default_buttoncb(v::MuJoCoViewer, s::WindowState, ev::ButtonEvent)
    p = v.phys
    m = p.model
    d = p.data
    ui = v.ui
    pert = p.pert

    if isright(ev.button) || isleft(ev.button)
        # set/unset pertubation on previously selected body
        if ispress(ev.action) && s.control && pert.select > 0
            newpert = s.right ? Int(LibMuJoCo.mjPERT_TRANSLATE) : Int(LibMuJoCo.mjPERT_ROTATE)
            # perturbation onset: reset reference
            iszero(pert.active) && mjv_initPerturb(m.internal_pointer, d.internal_pointer, ui.scn.internal_pointer, pert.internal_pointer)
            pert.active = newpert
        elseif isrelease(ev.action)
            # stop pertubation
            pert.active = 0
        end
    end

    if ev.isdoubleclick
        # new select: find geom and 3D click point, get corresponding body

        # stop perturbation on new select
        pert.active = 0

        # find the selected body
        selpnt = zeros(MVector{3,Float64})
        selgeom, selskin = Ref(Cint(0)), Ref(Cint(0))
        selbody = mjv_select(
            m.internal_pointer,
            d.internal_pointer,
            ui.vopt.internal_pointer,
            s.width / s.height,
            s.x / s.width,
            (s.height - s.y) / s.height,
            ui.scn.internal_pointer,
            selpnt,
            selgeom,
            selskin,
        )

        if isleft(ev.button) && nomod(s)
            if selbody >= 0
                # set body selection (including world body)

                # compute localpos
                tmp = selpnt - d.xpos[:, selbody+1] # TODO
                pert.localpos .= reshape(d.xmat[:, selbody+1], 3, 3) * tmp

                # record selection
                pert.select = selbody
                pert.skinselect = selskin
            else
                # no body selected, unset selection
                pert.select = 0
                pert.skinselect = -1
            end
        elseif isright(ev.button)
            # set camera to look at selected body (including world body)
            selbody >= 0 && (ui.cam.lookat .= selpnt)

            if s.control && selbody > 0
                # also set camera to track selected body (excluding world body)
                ui.cam.type = Int(LibMuJoCo.mjCAMERA_TRACKING)
                ui.cam.trackbodyid = selbody
                ui.cam.fixedcamid = -1
            end
        end
    end

    return
end

function default_scrollcb(v::MuJoCoViewer, s::WindowState, ev::ScrollEvent)
    m = v.phys.model
    mjv_moveCamera(m.internal_pointer, LibMuJoCo.mjMOUSE_ZOOM, 0.0, 0.05 * ev.dy, v.ui.scn.internal_pointer, v.ui.cam.internal_pointer)
    return
end

function handlers(v::MuJoCoViewer)
    return let v = v, ui = v.ui, p = v.phys
        [
            onevent(ButtonEvent) do s, ev
                default_buttoncb(e, s, ev)
            end,

            onevent(MouseMoveEvent) do s, ev
                default_mousemovecb(e, s, ev)
            end,


            onkey(GLFW.KEY_ESCAPE, what = "Quit") do s, ev
                ispress_or_repeat(ev.action) && (ui.shouldexit = true)
            end,

            # TODO: Implement printhelp()
            # onkey(GLFW.KEY_F1, what = "Show help message") do s, ev
            #     @warn "Not implemented yet"
            #     ispress_or_repeat(ev.action) && printhelp(v) 
            # end,

            onkey(GLFW.KEY_F2, what = "Toggle simulation info") do s, ev
                ispress_or_repeat(ev.action) && (ui.showinfo = !ui.showinfo)
            end,


            onkey(GLFW.KEY_F8, what = "Resize window to $RES_XGA (XGA)") do s, ev
                if ispress(ev.action) && v.ffmpeghandle === nothing
                    GLFW.SetWindowSize(s.window, RES_XGA...)
                end
            end,
            onkey(GLFW.KEY_F8, MOD_SHIFT, what = "Resize window to $RES_SXGA (SXGA)") do s, ev
                if ispress(ev.action) && v.ffmpeghandle === nothing
                    GLFW.SetWindowSize(s.window, RES_SXGA...)
                end
            end,

            onkey(GLFW.KEY_F9, what = "Resize window to $RES_HD (HD)") do s, ev
                if ispress(ev.action) && v.ffmpeghandle === nothing
                    GLFW.SetWindowSize(s.window, RES_HD...)
                end
            end,
            onkey(GLFW.KEY_F9, MOD_SHIFT, what = "Resize window to $RES_FHD (FHD)") do s, ev
                if ispress(ev.action) && v.ffmpeghandle === nothing
                    GLFW.SetWindowSize(s.window, RES_FHD...)
                end
            end,

            onkey(GLFW.KEY_F10, what = "Resize window to defaults") do s, ev
                if ispress(ev.action) && v.ffmpeghandle === nothing
                    GLFW.SetWindowSize(s.window, default_windowsize()...)
                end
            end,

            onkey(GLFW.KEY_F11, what = "Toggle fullscreen") do s, ev
                if ispress(ev.action) && v.ffmpeghandle === nothing
                    ismin = iszero(GLFW.GetWindowAttrib(s.window, GLFW.MAXIMIZED))
                    ismin ? GLFW.MaximizeWindow(s.window) : GLFW.RestoreWindow(s.window)
                end
            end,


            onscroll(what = "Zoom camera") do s, ev
                default_scrollcb(e, s, ev)
            end,

            onkey(GLFW.KEY_A, MOD_CONTROL, what = "Align camera scale") do s, ev
                ispress_or_repeat(ev.action) && alignscale!(ui, p.model)
            end,

            onkey(GLFW.KEY_V, MOD_CONTROL, what = "Toggle video recording") do s, ev
                if ispress(ev.action)
                    v.ffmpeghandle === nothing ? startrecord!(v) : stoprecord!(v)
                end
            end,

            # TODO: Implement this
            # onkey(GLFW.KEY_BACKSPACE, what = "Reset model") do s, ev
            #     ispress_or_repeat(ev.action) && reset!(p, mode(e))
            # end,

            # TODO: Implement this
            # onkey(GLFW.KEY_R, MOD_CONTROL, what = "Toggle reverse") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.reversed = !ui.reversed
            #         p.timer.rate *= -1
            #         resettime!(p)
            #     end
            # end,


            # TODO: Implement this
            # onkey(GLFW.KEY_SPACE, what = "Pause") do s, ev
            #     ispress_or_repeat(ev.action) && setpause!(ui, p, !ui.paused)
            # end,

            # TODO: Implement this
            # onevent(
            #     KeyEvent,
            #     when = describe(GLFW.KEY_RIGHT),
            #     what = "Step forward when paused (hold SHIFT for $(SHIFTSTEPSPERKEY) steps)"
            # ) do s, ev
            #     if ui.paused && ev.key === GLFW.KEY_RIGHT && ispress_or_repeat(ev.action)
            #         steps = s.shift ? 50 : 1
            #         for _ = 1:steps
            #             forwardstep!(p, mode(e))
            #         end
            #     end
            # end,

            # TODO: Implement this
            # onevent(
            #     KeyEvent,
            #     when = describe(GLFW.KEY_LEFT),
            #     what = "Step backwards when paused (hold SHIFT for $(SHIFTSTEPSPERKEY) steps)"
            # ) do s, ev
            #     if ui.paused && ev.key === GLFW.KEY_LEFT && ispress_or_repeat(ev.action)
            #         steps = s.shift ? 50 : 1
            #         for _ = 1:steps
            #             reversestep!(p, mode(e))
            #         end
            #     end
            # end,


            # TODO: Implement this
            # onkey(GLFW.KEY_ENTER, what = "Toggle speed mode") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.speedmode = !ui.speedmode
            #         setrate!(p.timer, ui.speedmode ? ui.speedfactor : 1)
            #     end
            # end,

            # TODO: Implement this
            # onkey(GLFW.KEY_UP, MOD_SHIFT, what = "Increase sim rate in speedmode") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.speedfactor *= 2
            #         setrate!(p.timer, ui.speedfactor)
            #     end
            # end,

            # TODO: Implement this
            # onkey(GLFW.KEY_DOWN, MOD_SHIFT, what = "Decrease sim rate in speedmode") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.speedfactor /= 2
            #         setrate!(p.timer, ui.speedfactor)
            #     end
            # end,


            # TODO: Do we want to include these?
            # onkey(GLFW.KEY_RIGHT, MOD_CONTROL, what = "Cycle engine mode forward") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         switchmode!(e, inc(e.curmodeidx, 1, length(e.modes)))
            #     end
            # end,

            # onkey(GLFW.KEY_LEFT, MOD_CONTROL, what = "Cycle engine mode backwards") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         switchmode!(e, dec(e.curmodeidx, 1, length(e.modes)))
            #     end
            # end,


            # TODO: Implement this
            # onkey(GLFW.KEY_MINUS, what = "Cycle label mode backwards") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.vopt[].label = dec(ui.vopt[].label, 0, Int(LibMuJoCo.mjNLABEL) - 1)
            #     end
            # end,

            # TODO: Implement this
            # onkey(GLFW.KEY_EQUAL, what = "Cycle label mode forward") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.vopt[].label = inc(ui.vopt[].label, 0, Int(LibMuJoCo.mjNLABEL) - 1)
            #     end
            # end,


            # TODO: Implement this
            # onkey(GLFW.KEY_LEFT_BRACKET, what = "Cycle frame mode backwards") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.vopt[].frame = dec(ui.vopt[].frame, 0, Int(LibMuJoCo.mjNFRAME) - 1)
            #     end
            # end,

            # TODO: Implement this
            # onkey(GLFW.KEY_RIGHT_BRACKET, what = "Cycle frame mode forward") do s, ev
            #     if ispress_or_repeat(ev.action)
            #         ui.vopt[].frame = inc(ui.vopt[].frame, 0, Int(LibMuJoCo.mjNFRAME) - 1)
            #     end
            # end,


            gen_mjflag_handlers(ui)...
        ]
    end
end

# TODO: Implement this
# function gen_mjflag_handlers(ui::UIState)
#     handlers = EventHandler[]
#     let vopt = ui.vopt, scn = ui.scn
#         for i=1:Int(LibMuJoCo.mjNVISFLAG)
#             key = glfw_lookup_key(LibMuJoCo.mjVISSTRING[3, i])
#             name = LibMuJoCo.mjVISSTRING[1, i]
#             h = onkey(key, what = "Toggle $name Viz Flag") do s, ev
#                 ispress_or_repeat(ev.action) && (vopt[].flags = _toggle(vopt[].flags, i))
#             end
#             push!(handlers, h)
#         end

#         for i=1:Int(LibMuJoCo.mjNRNDFLAG)
#             key = glfw_lookup_key(LibMuJoCo.mjRNDSTRING[3, i])
#             name = LibMuJoCo.mjRNDSTRING[1, i]
#             h = onkey(key, what = "Toggle $name Render Flag") do s, ev
#                 ispress_or_repeat(ev.action) && (scn[].flags = _toggle(scn[].flags, i))
#             end
#             push!(handlers, h)
#         end


#         n = LibMuJoCo.mjNGROUP

#         h = onevent(KeyEvent, when = "[1-$n]", what = "Toggle Group Groups 1-$n") do s, ev
#             i = Int(ev.key) - Int('0')
#             if ispress_or_repeat(ev.action) && iszero(modbits(s)) && checkbounds(Bool, vopt[].geomgroup, i)
#                 vopt[].geomgroup = _toggle(vopt[].geomgroup, i)
#             end
#         end
#         push!(handlers, h)

#         h = onevent(KeyEvent, when = "SHIFT+[1-$n]", what = "Toggle Site Groups 1-$n") do s, ev
#             i = Int(ev.key) - Int('0')
#             if ispress_or_repeat(ev.action) && isshift(modbits(s)) && checkbounds(Bool, vopt[].sitegroup, i)
#                 vopt[].sitegroup = _toggle(vopt[].sitegroup, i)
#             end
#         end
#         push!(handlers, h)
#     end
#     return handlers
# end

@inline function _toggle(A::SVector{N,LibMuJoCo.mjtByte}, i::Integer) where {N}
    A = MVector(A)
    A[i] = ifelse(A[i] > 0, 0, 1)
    SVector(A)
end