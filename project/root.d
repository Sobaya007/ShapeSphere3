import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;

mixin(Register!(root));

void root(Project proj, EventContext context) {

    setupWindow(proj);
    setupCanvas(proj);
    setupCamera(proj);
    setupConsole(proj);

    auto window = proj.get!Window("window");
    auto canvas = proj.get!Canvas("canvas");

    with (context()) {
        when(Frame).then({
            with (canvas.getContext()) {
                clear(ClearMode.Color, ClearMode.Depth);
            }
        });

        when((Ctrl + KeyButton.KeyR).pressed).then({
            auto oldTitle = window.title;
            window.title = "reloading...";
            proj.reload();
            window.title = oldTitle;
        });
    }

    proj.loadErrorHandler = (Exception e) {
        with (Log()) {
            writeln(e.msg);
        }
        import std.stdio : writeln;
        writeln(e.msg);
    };

    proj.load();
}

private void setupWindow(Project proj) {
    auto window = Window.getCurrentWindow();
    auto videoMode = Screen.getPrimaryScreen().currentVideoMode;
    window.pos = [0.pixel, 0.pixel];
    window.size = [videoMode.width.pixel/2, videoMode.height.pixel-200.pixel];

    proj["window"] = window;
}

private void setupCanvas(Project proj) {
    auto window = Window.getCurrentWindow();
    auto videoMode = Screen.getPrimaryScreen().currentVideoMode;
    with (CanvasBuilder()) {
        color.enable = true;
        color.clear = Color.White;
        depth.enable = true;
        size = [videoMode.width.pixel/2, videoMode.height.pixel-200.pixel];
        proj["canvas"] = build(window);
    }
}

private void setupCamera(Project proj) {
    with (PerspectiveCamera.Builder()) {
        near = 0.1;
        far = 200;
        fov = 60.deg;
        aspect = 1;

        auto camera = build();
        camera.pos = vec3(0);
        proj["camera"] = cast(Camera)camera;

        auto control = new CameraControl(camera);
        control.speed *= 4;
        proj["cameraControl"] = control;
    }
}

void setupConsole(Project proj) {
    auto console = new Console(proj);
    proj["console"] = console;

    auto canvas = proj.get!Canvas("canvas");
    auto consoleControl = new ConsoleControl(canvas, console);
    proj["consoleControl"] = consoleControl;
}
