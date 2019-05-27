module project.stage.stage;

import sbylib.wrapper.assimp : Scene, Node, Mesh, Face;
import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import sbylib.wrapper.assimp : PostProcessFlag;
import root;
import project.player.player;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {

    auto camera = proj.get!Camera("camera");
    auto canvas = proj.get!Canvas("canvas");

    with (context()) {
        auto modelForRender = StageModel.load("resource/castle.blend", PostProcessFlag.Triangulate);
        auto modelForCollision = StageModel.load("resource/castle.blend", PostProcessFlag.Triangulate);
        proj["stage"] = modelForCollision;

        with (AABBSet.Builder()) {
            foreach (polygon; modelForCollision.polygons) {
                add(polygon);
            }
            modelForCollision.polygonSet = build();
        }
        
        when(Frame).then({
            with (canvas.getContext()) {
                import sbylib.wrapper.gl : GlFunction, Capability, FaceMode;
                GlFunction().enable(Capability.CullFace);
                GlFunction().cullFace(FaceMode.Front);
                camera.capture(modelForRender);
                GlFunction().disable(Capability.CullFace);
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

ModelPolygon[] polygons(Model model) {
    return polygons(model.scene.rootNode, model.scene, mat4.identity);
}

ModelPolygon[] polygons(Node node, Scene scene, mat4 frameMatrix) {
    if (node.name == "table") return null;
    if (node.name == "shelf") return null;

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
        result ~= new ModelPolygon(face.indices.map!(i => (frameMatrix * vec4(mesh.vertices[i],1)).xzy).array, name);
    }
    return result;
}

auto mergeFaces(Mesh mesh) {
    import std.algorithm : map;
    import std.array : array;

    auto parent = new size_t[mesh.faces.length];

    foreach (i; 0..parent.length) parent[i] = i;

    size_t root(size_t x) {
        if (parent[x] == x) return x;
        return parent[x] = root(parent[x]);
    }

    void unite(size_t x, size_t y) {
        const rx = root(x);
        const ry = root(y);
        if (rx == ry) return;
        parent[rx] = ry;
    }

    bool same(size_t x, size_t y) {
        return root(x) == root(y);
    }

    foreach (i; 0..parent.length) {
        foreach (j; i+1..parent.length) {
            if (same(i,j)) continue;
            if (canMerge(mesh.vertices, mesh.faces[i], mesh.faces[j])) {
                unite(i,j);
            }
        }
    }

    Face[][size_t] result;
    foreach (i; 0..parent.length) {
        auto key = root(i);
        if (key !in result) result[key] = [];
        result[key] ~= mesh.faces[i];
    }

    return result.values.map!(faces => merge(mesh.vertices, faces)).array;
}

bool canMerge(VS)(VS vertices, Face face0, Face face1) {
    import std.algorithm : cartesianProduct, count;
    const connected = cartesianProduct(face0.indices, face1.indices).count!(pair => pair[0] == pair[1]) >= 2;
    const hasSameNormal = approxEqual(calcNormal(vertices, face0), calcNormal(vertices, face1));
    return connected && hasSameNormal;
}

vec3 calcNormal(VS)(VS vertices, Face face) {
    return normalize(cross(
        vertices[face.indices[0]] - vertices[face.indices[1]],
        vertices[face.indices[1]] - vertices[face.indices[2]]));
}

auto merge(VS)(VS vertices, Face[] faces) {
    import std.algorithm : map, sort, uniq;
    import std.array : join, array, popBack;
    import std.math : approxEqual, abs;
    import std.range : retro;

    auto indices = faces.map!(face => face.indices.dup).join.sort.uniq.array;

    auto normal = calcNormal(vertices, faces[0]);
    auto base1 = abs(normal.z).approxEqual(1) ? vec3(0,0,1) : vec3(0,1,0);
    auto base2 = normalize(cross(normal, base1));

    // Andrew's algorithm

    // sort indices by base2
    indices = indices.sort!((a,b) => dot(vertices[a], base2) < dot(vertices[b], base2)).array;

    // create upper side
    auto upper = indices[0..2];
    foreach (i; 2..indices.length) {
        while (upper.length >= 2 &&
                dot(cross(vertices[upper[$-1]] - vertices[upper[$-2]],
            vertices[indices[i]] - vertices[upper[$-1]]), normal) < 0) {
            upper.popBack();
        }
        upper ~= indices[i];
    }

    auto lower = indices[0..2];
    foreach (i; 2..indices.length) {
        while (lower.length >= 2 &&
                dot(cross(vertices[lower[$-1]] - vertices[lower[$-2]],
            vertices[indices[i]] - vertices[lower[$-1]]), normal) > 0) {
            lower.popBack();
        }
        lower ~= indices[i];
    }

    return upper[0..$-1] ~ lower.retro.array[0..$-1];
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
