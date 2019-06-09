module project.stage.collision;

import sbylib.wrapper.assimp : Assimp, Scene, Node, Mesh, Face;
import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import sbylib.wrapper.assimp : PostProcessFlag;
import root;
import project.player.player;
import project.npc.npc;

void setupStageCollision(Project proj, EventContext context) {
    with (context()) {
        auto npc = proj.get!(NPC[])("NPC");
        assert(npc);

        Assimp.initialize();
        auto scene = Scene.fromFile("resource/castle.blend", PostProcessFlag.Triangulate);
        auto model = new StageModel();
        proj["stage"] = model;

        with (AABBSet.Builder()) {
            foreach (polygon; polygons(scene.rootNode, scene, mat4.identity)) {
                add(polygon);
            }
            foreach (n; npc) {
                add(n);
            }
            model.polygonSet = build();
        }
    }
}

struct Person {
    vec3 pos;
}

Person[] findPerson(Node node) {
    import std : canFind;
    Person[] result;
    if (node.name.canFind("Person")) {
        result ~= Person(node.transformation.getTranslation.xzy);
    }
    foreach (child; node.children) {
        result ~= child.findPerson();
    }
    return result;
}

class StageModel { 
    AABBSet polygonSet;
}

ModelPolygon[] polygons(Node node, Scene scene, mat4 frameMatrix) {
    frameMatrix = node.transformation * frameMatrix;
    ModelPolygon[] result;
    if (node.name == "Castle" || node.name == "Ground" || node.name == "House" || node.name == "Bridge")
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
