module project.stage.render;

import sbylib.wrapper.assimp : Scene, Node, Mesh, Face;
import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import sbylib.wrapper.assimp : PostProcessFlag;
import sbylib.wrapper.gl : GlFunction, Capability, FaceMode;
import root;
import project.player.player;
import project.npc.npc;

void setupStageRender(Project proj, EventContext context) {

    auto camera = proj.get!Camera("camera");
    auto canvas = proj.get!Canvas("canvas");
    assert(camera);
    assert(canvas);

    with (context()) {
        auto model = StageModel.load("resource/castle.blend", PostProcessFlag.Triangulate);
        auto sky = new Sky;

        import std : map, array;

        auto npc = model.scene.rootNode.findPerson.map!((r) {
            auto npc = new NPC(proj, r.name);
            npc.pos = r.pos;
            return npc;
        }).array;
        proj["NPC"] = npc;
        
        proj["renderGameStage"] = {
            import sbylib.wrapper.gl : GlUtils;
            with (canvas.getContext()) {
                canvas.color.clear = Color.Black;
                clear(ClearMode.Color, ClearMode.Depth);
                camera.capture(sky);

                GlFunction().enable(Capability.CullFace);
                GlFunction().cullFace(FaceMode.Front);
                camera.capture(model);
                GlFunction().disable(Capability.CullFace);

                if (auto renderPlayer = proj.get!(void delegate())("renderPlayer")) {
                    renderPlayer();
                }
                foreach (n; npc) {
                    camera.capture(n);
                }
            }
        };
    }
}

struct Person {
    vec3 pos;
    string name;
}

Person[] findPerson(Node node) {
    import std : startsWith;
    Person[] result;
    if (node.name.startsWith("Person_")) {
        result ~= Person(node.transformation.getTranslation.xzy,
            node.name["Person_".length..$]);
    }
    foreach (child; node.children) {
        result ~= child.findPerson();
    }
    return result;
}

class StageModel : Model {
    mixin Material!(
            MaterialPattern!(StageMaterial, ".*"));
    mixin ImplLoad;
}

class StageMaterial : Material {
    mixin VertexShaderSource!q{
        #version 450

        in vec3 position;

        uniform mat4 frameMatrix;

        void main() {
            gl_Position = (frameMatrix * vec4(position, 1)).xzyw;
        }
    };

    mixin GeometryShaderSource!q{
        #version 450

        layout (triangles) in;
        layout (triangle_strip, max_vertices = 3) out;
        //layout (line_strip, max_vertices=6) out;

        out vec3 viewPosition;
        out vec3 worldNormal;

        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;

        void main() {
            vec3 normal = -normalize(cross(
                        gl_in[0].gl_Position.xyz - gl_in[1].gl_Position.xyz,
                        gl_in[1].gl_Position.xyz - gl_in[2].gl_Position.xyz));
            for (int i = 0; i < 3; i++) {
                gl_Position = projectionMatrix * viewMatrix * gl_in[(i+0)%3].gl_Position; 
                viewPosition = (viewMatrix * gl_in[(i+0)%3].gl_Position).xyz;
                worldNormal = normal;
                EmitVertex();
                //gl_Position = projectionMatrix * viewMatrix * gl_in[(i+1)%3].gl_Position; 
                //worldNormal = normal;
                //EmitVertex();
            }

            EndPrimitive();
        }
    };

    mixin FragmentShaderSource!q{
        #version 450

        in vec3 viewPosition;
        in vec3 worldNormal;
        out vec4 fragColor;

        void main() {
            float dist = length(viewPosition) / 200;
            vec3 color = normalize(worldNormal) * .5 + .5;
            vec3 result = mix(vec3(1), color, exp(-dist));
            fragColor = vec4(result, 1);
        }
    };
}

class Sky : Entity {
    mixin Material!(SkyMaterial);
    mixin ImplUniform;

    this() {
        import sbylib.wrapper.freeimage : FImage = Image;
        import sbylib.wrapper.gl : TextureBuilder, TextureFormat, TextureInternalFormat;

        this.geometry = GeometryLibrary().buildPlane();

        auto image = FImage.load("resource/sky.jpg");
        with (TextureBuilder()) {
            width = image.width;
            height = image.height;
            format = TextureFormat.BGR;
            iformat = TextureInternalFormat.RGB;
            tex = build(image.dataArray);
        }
        depthWrite = false;
    }
}

class SkyMaterial : Material {
    mixin VertexShaderSource!q{
        #version 450
        in vec4 position;
        out vec3 dir;

        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;

        void main() {
            gl_Position = vec4(position.xy * 2, 0, 1);
            vec2 projCoord = position.xy * 2;

            vec4 projCoord0 = vec4(projCoord * 1, 0, 1);
            vec4 projCoord1 = vec4(projCoord * 2, 0, 2);

            vec4 viewCoord0 = inverse(projectionMatrix) * projCoord0;
            vec4 viewCoord1 = inverse(projectionMatrix) * projCoord1;

            vec4 worldCoord0 = inverse(viewMatrix) * viewCoord0;
            vec4 worldCoord1 = inverse(viewMatrix) * viewCoord1;

            dir = normalize((worldCoord1 - worldCoord0).xyz);
        }
    };

    mixin FragmentShaderSource!q{
        #version 450
        in vec3 dir;
        out vec4 fragColor;

        uniform sampler2D tex;

        void main() {
            vec2 uv = (dir.xz + 1) * 0.5;
            fragColor = texture2D(tex, uv);
            fragColor = vec4(dir * .5 + .5, 1);
        }
    };
}
