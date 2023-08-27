# Adapted from https://github.com/Lyceum/LyceumMuJoCoViz.jl

@enum Mod::UInt16 begin
    MOD_ALT = GLFW.MOD_ALT
    MOD_SHIFT = GLFW.MOD_SHIFT
    MOD_CONTROL = GLFW.MOD_CONTROL
    MOD_SUPER = GLFW.MOD_SUPER
end

const DOUBLECLICK_THRESHOLD = 0.250

const PUNCTUATION = let
    chars = [
        GLFW.KEY_APOSTROPHE,
        GLFW.KEY_COMMA,
        GLFW.KEY_MINUS,
        GLFW.KEY_PERIOD,
        GLFW.KEY_SLASH,
        GLFW.KEY_SEMICOLON,
        GLFW.KEY_EQUAL,
        GLFW.KEY_LEFT_BRACKET,
        GLFW.KEY_BACKSLASH,
        GLFW.KEY_RIGHT_BRACKET,
        GLFW.KEY_GRAVE_ACCENT,
    ]
    Set{Int}(map(Int, chars))
end

const PRINTABLE_KEYS = let
    keys = [
        [i for i in Int(GLFW.KEY_A):Int(GLFW.KEY_Z)]...,
        [i + Int('0') for i=0:9]...,
    ]
    Set{Int}(keys)
end

function SetWindowAttrib(window::Window, attrib::Integer, value::Integer)
    ccall(
        (:glfwSetWindowAttrib, GLFW.libglfw),
        Cvoid,
        (Window, Cint, Cint),
        window,
        attrib,
        value,
    )
end

@inline function GetRefreshRate(monitor::GLFW.Monitor=GLFW.GetPrimaryMonitor())
    return GLFW.GetVideoMode(monitor).refreshrate
end

function default_windowsize()
    vmode = GLFW.GetVideoMode(GLFW.GetPrimaryMonitor())
    w, h = vmode.width, vmode.height
    return (trunc(Int, 2 * w / 3), trunc(Int, 2 * h / 3))
end

function create_window(width::Integer, height::Integer, title::String)
    GLFW.WindowHint(GLFW.SAMPLES, 4)
    GLFW.WindowHint(GLFW.VISIBLE, 0)
    window = GLFW.CreateWindow(width, height, title)
    GLFW.MakeContextCurrent(window)
    GLFW.SwapInterval(1)
    return window
end

@inline modbits(ms::Tuple{Vararg{Mod}}) = mapreduce(Cint, |, ms)
@inline modbits(ms::Mod...) = modbits(ms)

@inline isleft(b::MouseButton) = b === GLFW.MOUSE_BUTTON_LEFT
@inline ismiddle(b::MouseButton) = b === GLFW.MOUSE_BUTTON_MIDDLE
@inline isright(b::MouseButton) = b === GLFW.MOUSE_BUTTON_RIGHT

@inline isrelease(action::Action) = action === GLFW.RELEASE
@inline ispress(action::Action) = action === GLFW.PRESS
@inline isrepeat(action::Action) = action === GLFW.REPEAT
@inline ispress_or_repeat(action::Action) = ispress(action) || isrepeat(action)

@inline isalt(mods::Cint) = (mods & GLFW.MOD_ALT) == GLFW.MOD_ALT
@inline isshift(mods::Cint) = (mods & GLFW.MOD_SHIFT) == GLFW.MOD_SHIFT
@inline iscontrol(mods::Cint) = (mods & GLFW.MOD_CONTROL) == GLFW.MOD_CONTROL
@inline issuper(mods::Cint) = (mods & GLFW.MOD_SUPER) == GLFW.MOD_SUPER

@inline isalt(key::Key) = key === GLFW.KEY_LEFT_ALT || key === GLFW.KEY_RIGHT_ALT
@inline isshift(key::Key) = key === GLFW.KEY_LEFT_SHIFT || key === GLFW.KEY_RIGHT_SHIFT
@inline iscontrol(key::Key) = key === GLFW.KEY_LEFT_CONTROL || key === GLFW.KEY_RIGHT_CONTROL
@inline issuper(key::Key) = key === GLFW.KEY_LEFT_SUPER || key === GLFW.KEY_RIGHT_SUPER

function glfw_lookup_key(x::Integer)
    for key in instances(Key)
        Int(key) == x && return key
    end
    error("Key with unicode value $x not found")
end
@inline glfw_lookup_key(s::AbstractString) = glfw_lookup_key(str2unicode(s))

function describe(x::Mod)
    if x === MOD_CONTROL
        return "CTRL"
    else
        s = String(Symbol(x))
        return last(split(s, '_'))
    end
end

function describe(x::Key)
    i = Int(x)
    c = Char(i)

    if i in PUNCTUATION
        return "\"$(c)\""
    elseif i in PRINTABLE_KEYS
        return string(c)
    elseif x === GLFW.KEY_ESCAPE
        return "ESC"
    else
        s = String(Symbol(x))
        return last(split(s, "KEY_"))
    end
end

function describe(x::MouseButton)
    x === GLFW.MOUSE_BUTTON_LEFT && return "Left Click"
    x === GLFW.MOUSE_BUTTON_MIDDLE && return "Middle Click"
    x === GLFW.MOUSE_BUTTON_RIGHT && return "Right Click"
    error("unknown button $x")
end

function describe(xs::Union{Key,MouseButton,Mod}...)
    ms = sort!([describe(x) for x in xs if x isa Mod])
    ks = sort!([describe(x) for x in xs if x isa Key])
    bs = sort!([describe(x) for x in xs if x isa MouseButton])
    return join(vcat(ms, ks, bs), "+")
end


####
#### Events
####

abstract type Event end

struct KeyEvent <: Event
    key::Key
    action::Action
    time::Float64
end

struct ButtonEvent <: Event
    button::MouseButton
    action::Action
    isdoubleclick::Bool
    time::Float64
end

struct MouseMoveEvent <: Event
    dx::Float64
    dy::Float64
    isdrag::Bool
    time::Float64
end

struct ScrollEvent <: Event
    dx::Float64
    dy::Float64
    time::Float64
end

struct WindowResizeEvent <: Event
    dx::Float64
    dy::Float64
    time::Float64
end


####
#### Window manager
####

mutable struct WindowState
    x::Float64
    y::Float64
    sx::Float64
    sy::Float64

    left::Bool
    middle::Bool
    right::Bool
    lastbuttonevent::Maybe{ButtonEvent}
    lastbuttonpress::Maybe{ButtonEvent}

    alt::Bool
    shift::Bool
    control::Bool
    super::Bool
    lastkeyevent::Maybe{KeyEvent}

    width::Float64
    height::Float64
    window::Window

    function WindowState(win::Window)
        x, y = GLFW.GetCursorPos(win)
        width, height = GLFW.GetWindowSize(win)
        new(
            x,
            y,
            0,
            0,

            GLFW.GetMouseButton(win, GLFW.MOUSE_BUTTON_LEFT),
            GLFW.GetMouseButton(win, GLFW.MOUSE_BUTTON_MIDDLE),
            GLFW.GetMouseButton(win, GLFW.MOUSE_BUTTON_RIGHT),
            nothing,
            nothing,

            GetKey(win, GLFW.KEY_LEFT_ALT) || GetKey(win, GLFW.KEY_RIGHT_ALT),
            GetKey(win, GLFW.KEY_LEFT_SHIFT) || GetKey(win, GLFW.KEY_RIGHT_SHIFT),
            GetKey(win, GLFW.KEY_LEFT_CONTROL) || GetKey(win, GLFW.KEY_RIGHT_CONTROL),
            GetKey(win, GLFW.KEY_LEFT_SUPER) || GetKey(win, GLFW.KEY_RIGHT_SUPER),
            nothing,

            width,
            height,
            win,
        )
    end
end

@inline function ispressed(s::WindowState, button::MouseButton)
    button === GLFW.MOUSE_BUTTON_LEFT && return s.left
    button === GLFW.MOUSE_BUTTON_MIDDLE && return s.middle
    button === GLFW.MOUSE_BUTTON_RIGHT && return s.right
    return GLFW.GetMouseButton(s.window, button)
end

@inline function modbits(s::WindowState)
    m = Cint(0)
    s.alt && (m |= Integer(MOD_ALT))
    s.shift && (m |= Integer(MOD_SHIFT))
    s.control && (m |= Integer(MOD_CONTROL))
    s.super && (m |= Integer(MOD_SUPER))
    return m
end

@inline nomod(s::WindowState) = iszero(modbits(s))

struct ObsEntry{E<:Event}
    state::WindowState
    event::E
end

const Obs{E} = Observable{Maybe{ObsEntry{E}}}
struct WindowEvents
    key::Obs{KeyEvent}
    button::Obs{ButtonEvent}
    mouse::Obs{MouseMoveEvent}
    scroll::Obs{ScrollEvent}
    windowresize::Obs{WindowResizeEvent}

    function WindowEvents()
        new(
            Obs{KeyEvent}(nothing),
            Obs{ButtonEvent}(nothing),
            Obs{MouseMoveEvent}(nothing),
            Obs{ScrollEvent}(nothing),
            Obs{WindowResizeEvent}(nothing),
        )
    end
end

eventtype(x::Union{Obs{E},ObsEntry{E}}) where {E} = E

events(x::WindowEvents) = ntuple(i -> getfield(x, i), Val(fieldcount(WindowEvents)))

mutable struct WindowManager
    state::WindowState
    events::WindowEvents
    function WindowManager(window::Window)
        mngr = new(WindowState(window), WindowEvents())
        attachcallbacks!(mngr, window)
    end
end

function attachcallbacks!(mngr::WindowManager, window::Window)
    let mngr = mngr
        GLFW.SetKeyCallback(window, (w, k, s, a, m) -> keycb!(mngr, k, s, a, m))
        GLFW.SetCursorPosCallback(window, (w, x, y) -> cursorposcb!(mngr, x, y))
        GLFW.SetMouseButtonCallback(window, (w, b, a, m) -> mousebuttoncb!(mngr, b, a, m))
        GLFW.SetScrollCallback(window, (w, x, y) -> scrollcb!(mngr, x, y))
        GLFW.SetWindowSizeCallback(window, (w, x, y) -> windowsizecb!(mngr, x, y))
    end
    return mngr
end

function trigger!(mngr::WindowManager, e::Event)
    @debug e
    entry = ObsEntry(mngr.state, e)
    s, events = mngr.state, mngr.events

    if e isa KeyEvent
        events.key[] = entry
    elseif e isa ButtonEvent
        events.button[] = entry
    elseif e isa MouseMoveEvent
        events.mouse[] = entry
    elseif e isa ScrollEvent
        events.scroll[] = entry
    elseif e isa WindowResizeEvent
        events.windowresize[] = entry
    else
        error("Unknown event $e")
    end

    return mngr
end

function keycb!(mngr::WindowManager, key::Key, ::Cint, action::Action, mods::Cint)
    s = mngr.state
    status = ispress(action) || isrepeat(action)

    if isalt(key)
        s.alt = status
    elseif isshift(key)
        s.shift = status
    elseif iscontrol(key)
        s.control = status
    elseif issuper(key)
        s.super = status
    else
        s.alt = isalt(mods)
        s.shift = isshift(mods)
        s.control = iscontrol(mods)
        s.super = issuper(mods)
    end

    ev = KeyEvent(key, action, time())
    trigger!(mngr, ev)
    s.lastkeyevent = ev

    return
end

function cursorposcb!(mngr::WindowManager, x, y)
    s = mngr.state

    t = time()
    dx = x - s.x
    dy = y - s.y
    s.x = x
    s.y = y
    isdrag = !isnothing(s.lastbuttonevent) && ispressed(s, s.lastbuttonevent.button)

    trigger!(mngr, MouseMoveEvent(dx, dy, isdrag, t))

    return
end

function mousebuttoncb!(
    mngr::WindowManager,
    button::MouseButton,
    action::Action,
    mods::Cint,
)
    s, events = mngr.state, mngr.events
    t = time()

    lastbpress= mngr.state.lastbuttonpress
    isdoubleclick = (
        lastbpress !== nothing
        && ispress(action)
        && button === lastbpress.button
        && (t - lastbpress.time) < DOUBLECLICK_THRESHOLD
    )

    if isleft(button)
        s.left = ispress(action)
    elseif ismiddle(button)
        s.middle = ispress(action)
    elseif isright(button)
        s.right = ispress(action)
    end

    ev = ButtonEvent(button, action, isdoubleclick, t)
    trigger!(mngr, ev)
    s.lastbuttonevent = ev
    ispress(action) && (s.lastbuttonpress = ev)

    return
end

function scrollcb!(mngr::WindowManager, dx, dy)
    mngr.state.sx += dx
    mngr.state.sy += dy
    trigger!(mngr, ScrollEvent(dx, dy, time()))
    return
end

function windowsizecb!(mngr::WindowManager, width, height)
    s = mngr.state
    dx = width - s.width
    dy = height - s.height
    s.width = width
    s.height = height
    trigger!(mngr, WindowResizeEvent(dx, dy, time()))
    return
end

####
#### Event handlers
####

struct EventHandler{E<:Event}
    callback
    what::Maybe{String}
    when::Maybe{String}
    function EventHandler{E}(cb, what, when) where {E<:Event}
        if !hasmethod(cb, (WindowState, Event))
            error("EventHandler callbacks must have signature (WindowState, Event), got: $(methods(f))")
        end
        if what === nothing && when !== nothing
            error("Must provide `what` if specifying `when`")
        end
        new{E}(cb, what, when)
    end
end
function EventHandler{E}(cb; what = nothing, when = nothing) where {E<:Event}
    EventHandler{E}(cb, what, when)
end

eventtype(::EventHandler{E}) where {E} = E

(h::EventHandler)(x::ObsEntry) = h.callback(x.state, x.event)

function register!(mngr::WindowManager, hs::EventHandler...)
    for obs in events(mngr.events), h in hs
        if eventtype(h) === eventtype(obs)
            on(h, obs)
        end
    end
    return mngr
end

function deregister!(mngr::WindowManager, hs::EventHandler...)
    for obs in events(mngr.events), h in hs
        if eventtype(h) === eventtype(obs)
            off(obs, h)
        end
    end
    return mngr
end

function onevent(cb, E::Type{<:Event}; what = nothing, when = nothing)
    return EventHandler{E}(cb, what, when)
end

function onkey(cb, key::Key; what = nothing)
    return let cb = cb
        EventHandler{KeyEvent}(what, describe(key)) do s, e
            e.key === key && iszero(modbits(s)) && cb(s, e)
        end
    end
end

function onkey(cb, key::Key, mods::Mod...; what = nothing)
    when = describe(key, mods...)
    return let cb = cb, mods = modbits(mods)
        EventHandler{KeyEvent}(what, when) do s, e
            e.key === key && modbits(s) === mods && cb(s, e)
        end
    end
end

function onscroll(cb; what = nothing)
    return let cb = cb
        EventHandler{ScrollEvent}(what, "Scroll") do s, e
            iszero(modbits(s)) && cb(s, e)
        end
    end
end

function onscroll(cb, mods::Mod...; what = nothing)
    when = "$(describe(mods...)) + Scroll"
    return let cb = cb, mods = modbits(mods)
        EventHandler{ScrollEvent}(what, when) do s, e
            modbits(s) === mods && cb(s, e)
        end
    end
end