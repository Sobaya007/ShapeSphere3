module project.scene.test;

import sbylib.graphics;
import sbylib.editor;

//mixin(Register!(setupTestScene));

void setupTestScene(Project proj, EventContext context) {
    auto camera = proj.get!(Camera)("camera");
    auto canvas = proj.get!(Canvas)("canvas");
    auto floor = setupFloor(proj);

    import project.npc.fukidashi;
    auto f = new Fukidashi(proj, floor, "Poyo");
    proj["renderScene"] = {
        with (canvas.getContext()) {
            canvas.color.clear = Color.Black;
            clear(ClearMode.Color, ClearMode.Depth);
            camera.capture(floor);
            camera.capture(f);
        }
    };
    proj.get!CameraControl("cameraControl").bind();
}

private auto setupFloor(Project proj) {
    Floor f;
    with (Floor.Builder()) {
        geometry = GeometryLibrary().buildPlane().transform(
                mat3.axisAngle(vec3(1,0,0), 90.deg) * mat3.scale(vec3(10)));
        f = build();
        f.pos = vec3(0,-2,0);
    }
    return f;
}

class FloorMaterial : Material {
    mixin VertexShaderSource!q{
        #version 450

        in vec4 position;
        in vec2 uv;
        out vec2 uv2;
        uniform mat4 worldMatrix;
        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;

        void main() {
            gl_Position = projectionMatrix * viewMatrix * worldMatrix * position;
            uv2 = uv;
        }
    };

    mixin FragmentShaderSource!q{
        #version 450

        in vec2 uv2;
        out vec4 fragColor;

        float value1() {
            const float size = 0.1 / 8;
            vec2 po = mod(uv2 / (size * 2), vec2(1)) - 0.5;
            if (po.x * po.y > 0) {
                return 0.2;
            } else {
                return 0.3;
            }
        }

        float value2() {
            const float size = 0.1 / 8;
            vec2 po = mod(uv2 / (size * 2), vec2(1)) - 0.5;
            if (po.x * po.y > 0) {
                return 0.2;
            } else {
                return 0.1;
            }
        }

        float value() {
            const float size = 0.1;
            vec2 po = mod(uv2 / (size * 2), vec2(1)) - 0.5;
            if (po.x * po.y > 0) {
                return value1();
            } else {
                return value2();
            }
        }

        void main() {
            fragColor = vec4(vec3(value()), 1);
        }
    };
}

class Floor : Entity {
    mixin ImplPos;
    mixin ImplRot;
    mixin ImplScale;
    mixin ImplWorldMatrix;
    mixin Material!(FloorMaterial);
    mixin ImplUniform;
    mixin ImplBuilder;
}
