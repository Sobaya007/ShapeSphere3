import sbylib.graphics;
import sbylib.editor;
import sbylib.wrapper.glfw;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {
    auto camera = proj.get!Camera("camera");

    with (context()) {
        with (Player.Builder()) {
            geometry = GeometryLibrary().buildIcosahedron(2);

            //auto player = build();
            //player.blend = true;
            //player.depthWrite = false;
            //when(Frame).then({
            //    camera.capture(player);
            //});
        }
    }
}

class Player : Entity {
    mixin ImplWorldMatrix;
    mixin Material!(PlayerMaterial);
    mixin ImplUniform;
    mixin ImplBuilder;
}

class PlayerMaterial : Material {
    mixin VertexShaderSource!q{
        #version 450

        in vec4 position;
        uniform mat4 worldMatrix;
        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;
        
        void main() {
            gl_Position = projectionMatrix * viewMatrix * worldMatrix * position;
        }
    };

    mixin FragmentShaderSource!q{
        #version 450

        out vec4 fragColor;
        
        void main() {
            fragColor = vec4(1,0,0,0.5);
        }
    };
}
