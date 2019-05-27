module project.player.player;

import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {
    auto camera = proj.get!Camera("camera");
    auto canvas = proj.get!Canvas("canvas");

    auto player = new Player();
    proj["player"] = player;
    with (context()) {
        when(Frame).then({
            with (canvas.getContext()) {
                camera.capture(player);
            }
        });
    }

    auto cameraControl = proj.get!CameraControl("cameraControl");
    auto consoleControl = proj.get!ConsoleControl("consoleControl");

    with (cameraControl()) {
        when((Ctrl + KeyButton.KeyP).pressed).then({
            cameraControl.unbind();
            consoleControl.bind();
        });
        when(KeyButton.Enter.pressed).then({
            cameraControl.unbind();
            player.bind();
        });
    }

    with (consoleControl()) {
        when(KeyButton.Escape.pressed).then({
            consoleControl.unbind();
            cameraControl.bind();
        });
    }

    with (player()) {
        when(KeyButton.BackSpace.pressed).then({
            player.unbind();
            cameraControl.bind();
        });
        when((Ctrl + KeyButton.KeyP).pressed).then({
            player.unbind();
            consoleControl.bind();
        });
    }

    player.bind();
}

class Player : Entity, CollisionCapsule {

    mixin Material!(PlayerMaterial);
    mixin ImplUniform;
    mixin ImplAABB;

    private static immutable {
        uint RECURSION_LEVEL = 2;
        float DEFAULT_RADIUS = 0.5;
    }

    public {
        import std.traits : ReturnType;

        alias PlayerGeometry = Geometry!Attribute;
        PlayerGeometry geom;
        Particle[] particleList;
        Particle[2][] pairList;
        float _radius = 0;
        vec3 pos = vec3(0);
        vec3 beforePos = vec3(0);
        EventContext context;
        alias context this;
    }

    this() {
        this.context = new EventContext;

        auto r = createGeometry();

        this.geometry = this.geom = r.geom;
        this.particleList = r.particleList;
        this.pairList = r.pairList;

        this.innerLevel = 3;
        this.outerLevel = 3;
        this.needleCount = 0;
    }

    float calcVolume(vec3 center) const {
        import std.math : abs;

        float volume = 0;
        foreach (i; 0..geom.indexList.length/3) {
            const a = particleList[geom.indexList[i*3+0]].position - center;
            const b = particleList[geom.indexList[i*3+1]].position - center;
            const c = particleList[geom.indexList[i*3+2]].position - center;
            volume += mat3(a,b,c).determinant;
        }
        return abs(volume) / 6;
    }

    float calcArea(vec3 center) const {
        float area = 0;
        foreach (i; 0..geom.indexList.length/3) {
            auto a = particleList[geom.indexList[i*3+0]].position - center;
            auto b = particleList[geom.indexList[i*3+1]].position - center;
            auto c = particleList[geom.indexList[i*3+2]].position - center;
            area += length(cross(a - b, a - c));
        }
        return area / 2;
    }

    float calcRadius(vec3 center) const {
        import std.algorithm : map, sum;
        return particleList.map!(a => length(a.position - center)).sum / particleList.length;
    }

    float calcMin(vec3 n) const {
        import std.algorithm : map, reduce, min;
        return particleList.map!(p => p.position.dot(n)).reduce!min;
    }

    float calcMax(vec3 n) const {
        import std.algorithm : map, reduce, max;
        return particleList.map!(p => p.position.dot(n)).reduce!max;
    }

    vec3 calcCenter() const {
        import std.algorithm : map, sum;

        return particleList.map!(p => p.position).sum / particleList.length;
    }

    vec3 calcLinearVelocity() const {
        import std.algorithm : map, sum;

        return particleList.map!(p => p.velocity).sum / particleList.length;
    }

    vec3 calcAngularVelocity() const {
        import std.algorithm : map, sum;

        const c = this.calcCenter();
        const l = this.calcLinearVelocity();
        return particleList.map!((p) {
            auto r = p.position - c;
            auto v = p.velocity - l;
            return cross(r, v) / lengthSq(r);
        }).sum / particleList.length;
    }

    override float radius() {
        return _radius;
    }

    override vec3[2] ends() {
        return [pos, beforePos];
    }

    static class Particle : CollisionCapsule {
        vec3 position; /* in World, used for Render */
        vec3 velocity;
        vec3 normal; /* in World */
        vec3 force;
        bool isNeedle;
        Particle[] next;
        vec3[2] _ends;
        vec3 beforePos;
        float timeOfImpact;
        vec3 contactNormal;
        size_t index;

        mixin ImplAABB;

        this(vec3 p) {
            static size_t idx;
            this.index = idx++;
            this.position = p;
            this.beforePos = p;
            this.index = index;
            this.normal = normalize(p);
            this.velocity = vec3(0);
            this.force = vec3(0,0,0);
            this._ends = [p,p];
        }

        override float radius() {
            return 0.1;
        }

        override vec3[2] ends() {
            return _ends;
        }
    }
}

class PlayerMaterial : Material {
    mixin PatchVertices!(3);

    mixin VertexShaderSource!(q{
        #version 450
        in vec3 position;
        in vec3 normal;
        in float isNeedle;
        out vec3 normal2;
        out float isNeedle2;

        void main() {
            gl_Position = vec4(position,1);
            normal2 = normal;
            isNeedle2 = isNeedle;
        }
    });

    mixin TessellationControlShaderSource!q{
        #version 450
        layout(vertices=3) out;
        in vec3 normal2[];
        in float isNeedle2[];
        out vec3 normal3[];
        out float isNeedle3[];
        uniform int outerLevel;
        uniform int innerLevel;

        void main(){
            gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
            normal3[gl_InvocationID] = normal2[gl_InvocationID];
            isNeedle3[gl_InvocationID] = isNeedle2[gl_InvocationID];
            gl_TessLevelOuter[0] = outerLevel;
            gl_TessLevelOuter[1] = outerLevel;
            gl_TessLevelOuter[2] = outerLevel;
            gl_TessLevelInner[0] = innerLevel;
        }
    };

    mixin TessellationEvaluationShaderSource!q{
        #version 450
        layout(triangles) in;
        in vec3 normal3[];
        in float isNeedle3[];
        out vec3 normal4;
        out float isNeedle4;

        uniform vec3 center;

        void main(){
            vec4 p = vec4(0);
            vec3 n = vec3(0);
            float isNeedle = 0;
            for (int i = 0; i < 3; i++) {
                p += gl_TessCoord[i] * gl_in[i].gl_Position;
                n += gl_TessCoord[i] * normal3[i];
                isNeedle += gl_TessCoord[i] * isNeedle3[i];
            }
            p.xyz = (p.xyz - center) / length(n) + center;

            gl_Position = p;
            normal4 = n;
            isNeedle4 = isNeedle;
        }
    };

    mixin GeometryShaderSource!q{
        #version 450
        layout(triangles) in;
        layout(triangle_strip,max_vertices=4) out;
        in vec3 normal4[];
        in float isNeedle4[];
        out vec3 pos;
        out vec3 n;
        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;
        uniform float needleCount;
        uniform vec3 center;
        
        void main(){
            for (int i = 0; i < 3; i++) {
                vec4 p = gl_in[i].gl_Position;
                float s = mix(1, isNeedle4[i] * 0.7 + 0.8, needleCount);
                p.xyz = (p.xyz - center) * s + center;
                gl_Position = projectionMatrix * viewMatrix * p;
                pos = (viewMatrix * p).xyz;
                n = (viewMatrix * vec4(normal4[i],0)).xyz;
                EmitVertex();
            }
        
            EndPrimitive();
        }
    };

    mixin FragmentShaderSource!(q{
        #version 450
        in vec3 pos;
        in vec3 n;
        out vec4 fragColor;

        const vec3 lightPos = vec3(1,3,1);

        // yawaraka orange
        void main() {
            vec3 nrm = normalize(n);
            vec3 camV = normalize(pos);
            vec3 col = vec3(1,0.5,0.1);
            vec3 acc = vec3(0);
        
            vec3 ligC = vec3(3,3,5),ligV;
            float fac,cus;
            for(int i=0;i<1;i++){
                ligV = normalize(lightPos-pos);
                fac = pow(length(lightPos-pos),0.9);
                cus = 0;
                cus += max(0.,dot(-camV,nrm))/4.;
                cus *= max(0.2,1.-pow(1.-abs(dot(-camV,nrm)),3.));
                cus += max(0.,dot(ligV,nrm)*0.5+0.5)/6.;
                cus += max(0.,dot(ligV,nrm))/6.;
        
                acc += cus*ligC*col*4.0/fac * 2.0;
            }
            fragColor.rgb = acc;
            fragColor.a = length(pos) * length(pos) * 0.1;
        }
    });
}

private struct Attribute {
    @transformable vec3 position;
    @transformable vec3 normal;
    float isNeedle;
}

private auto createGeometry() {
    with (GeometryBuilder!Attribute()) {
        auto r = createParticle();
        foreach (p; r.particleList) {
            with (p) {
                add(Attribute(position, normal, isNeedle ? 1 : 0));
            }
        }
        foreach (i; r.indexList) {
            select(i);
        }

        auto geom = build();
        geom.primitive = Primitive.Patch;

        struct Result {
            Geometry!Attribute geom;
            Player.Particle[] particleList;
            Player.Particle[2][] pairList;
        }
        return Result(geom, r.particleList, r.pairList);
    }
}

private auto createParticle() {
    import std.algorithm : map;
    import std.array : array;
    auto geom = GeometryLibrary().buildIcosahedron(Player.RECURSION_LEVEL)
        .transform(mat3.scale(vec3(Player.DEFAULT_RADIUS)));
    auto particleList = geom.attributeList
        .map!(a => new Player.Particle(a.position.xyz)).array;

    auto pairList = createPairList(geom.indexList, particleList);

    foreach (p; particleList) {
        import std.algorithm : all;
        p.isNeedle = p.next.all!(a => a.isNeedle == false);
    }

    struct Result {
        Player.Particle[] particleList;
        Player.Particle[2][] pairList;
        uint[] indexList;
    }

    return Result(particleList, pairList, geom.indexList);
}

private auto createPairList(uint[] indexList, Player.Particle[] particleList) {
    Player.Particle[2][] result;

    alias ID = uint[2];
    ID pairID(uint a,uint b) { return a < b ? [a,b] : [b,a]; }

    ID[] pairIDList;

    foreach (i; 0..indexList.length/3) {
        foreach (j; 0..3) {
            import std.algorithm : canFind;

            auto id = pairID(
                indexList[i*3+(j+0)%3],
                indexList[i*3+(j+1)%3]);

            if (pairIDList.canFind(id)) continue;
            pairIDList ~= id;

            auto p0 = particleList[id[0]];
            auto p1 = particleList[id[1]];
            result ~= [p0, p1];
            p0.next ~= p1;
            p1.next ~= p0;
        }
    }

    return result;
}

