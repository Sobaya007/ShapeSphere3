module project.stage.stage;

import sbylib.wrapper.assimp : Scene, Node, Mesh;
import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import root;
import project.player.player;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {

    auto camera = proj.get!Camera("camera");
    auto canvas = proj.get!Canvas("canvas");

    with (context()) {
        auto model = StageModel.load("resource/test.blend");
        proj["stage"] = model;

        with (AABBSet.Builder()) {
            foreach (polygon; model.polygons) {
                add(polygon);
            }
            model.polygonSet = build();
        }
        
        when(Frame).then({
            with (canvas.getContext()) {
                camera.capture(model);
            }
        });
    }
}

class StageModel : Model {
    mixin Material!(
            MaterialPattern!(StageMaterial, ".*"));
    mixin ImplLoad;

    AABBSet polygonSet;
}

class StageMaterial : Material {
    mixin VertexShaderSource!q{
        #version 450

        in vec3 position;

        uniform mat4 frameMatrix;

        void main() {
            gl_Position = frameMatrix * vec4(position.xzy, 1);
        }
    };

    mixin GeometryShaderSource!q{
        #version 450

        layout (triangles) in;
        //layout (triangle_strip, max_vertices = 3) out;
        layout (line_strip, max_vertices=6) out;

        out vec3 worldNormal;

        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;

        void main() {
            vec3 normal = -normalize(cross(
                        gl_in[0].gl_Position.xyz - gl_in[1].gl_Position.xyz,
                        gl_in[1].gl_Position.xyz - gl_in[2].gl_Position.xyz));
            for (int i = 0; i < 3; i++) {
                gl_Position = projectionMatrix * viewMatrix * gl_in[(i+0)%3].gl_Position; 
                worldNormal = normal;
                EmitVertex();
                gl_Position = projectionMatrix * viewMatrix * gl_in[(i+1)%3].gl_Position; 
                worldNormal = normal;
                EmitVertex();
            }

            EndPrimitive();
        }
    };

    mixin FragmentShaderSource!q{
        #version 450

        in vec3 worldNormal;
        out vec4 fragColor;

        void main() {
            fragColor = vec4(normalize(worldNormal) * .5 + .5, 1);
        }
    };
}

ModelPolygon[] polygons(Model model) {
    return polygons(model.scene.rootNode, model.scene, mat4.identity);
}

ModelPolygon[] polygons(Node node, Scene scene, mat4 frameMatrix) {
    frameMatrix = node.transformation * frameMatrix;
    ModelPolygon[] result;
    foreach (mesh; node.meshes) result ~= scene.meshes[mesh].polygons(scene, frameMatrix);
    foreach (child; node.children) result ~= child.polygons(scene, frameMatrix);
    return result;
}

ModelPolygon[] polygons(Mesh mesh, Scene scene, mat4 frameMatrix) {
    import std.algorithm : map, find;
    import std.array : array, front;

    auto material = scene.materials[mesh.materialIndex];
    auto name = material.properties.find!(p => p.key == "?mat.name").front.data!string;

    ModelPolygon[] result;
    foreach (face; mesh.faces) {
        result ~= new ModelPolygon(face.indices.map!(i => (frameMatrix * vec4(mesh.vertices[i].xzy,1)).xyz).array, name);
    }
    import std.algorithm : map, sort, group;
    import std.array : array;
    import std.stdio : writeln;
    mesh.faces.map!(face => face.indices.length).array.sort.group.writeln;
    return result;
}

class ModelPolygon : CollisionPolygon {

    private vec3[] _vertices;
    string materialName;
    
    this(const vec3[] _vertices, string materialName) {
        this._vertices = _vertices.dup;
        this.materialName = materialName;
    }

    override vec3[] vertices() {
        return _vertices;
    }

    mixin ImplAABB;
}
