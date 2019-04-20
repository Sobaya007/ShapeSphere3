import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {
    auto camera = proj.get!Camera("camera");

    with (context()) {
    }
}

class Player : Entity, CollisionCapsule {
}
