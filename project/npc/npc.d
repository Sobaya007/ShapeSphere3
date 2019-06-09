module project.npc.npc;

import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import std.datetime.stopwatch : StopWatch;
import project.player.player;
import project.npc.fukidashi;

class ConversationArea : CollisionSphere {

    mixin ImplAABB;

    private NPC npc;

    this(NPC npc) {
        this.npc = npc;
    }

    override float radius() {
        return 6;
    }

    override vec3 center() {
        return npc.pos;
    }
}

class NPC : Entity, CollisionSphere {

    mixin ImplPos;
    mixin ImplWorldMatrix;
    mixin Material!(NPCMaterial);
    mixin ImplUniform;
    mixin ImplAABB;

    typeof(GeometryLibrary.buildIcosahedron(2)) geom;

    this(Project proj, string name) {
        import std : map, filter, find, startsWith, strip, array, front, readText, split, pipe;
        this.geom = GeometryLibrary().buildIcosahedron(2);
        this.geom.primitive = Primitive.Patch;
        //auto particleList = geom.attributeList
            //.map!(a => new Player.Particle(a.position.xyz)).array;

        this.geometry = geom;
        this.innerLevel = 3;
        this.outerLevel = 3;

        auto area = new ConversationArea(this);

        auto player = proj.get!Player("player");
        auto camera = proj.get!Camera("camera");
        assert(player);
        assert(camera);

        auto serif = readText("resource/serif.txt").split("\n")
            .map!(line => line.split(":").map!(strip).array)
            .filter!(line => line.length == 2)
            .find!(info => info[0] == name)
            .map!(info => info[1])
            .pipe!(serif => serif.empty ? "<No serif specified>" : serif.front);
        Fukidashi f;

        when(this.beforeRender).then({
            if (f) camera.capture(f);
        });
        auto g = (Player p, ConversationArea a, CapsuleSphereResult r) {
            if (f) return;
            f = new Fukidashi(proj, this, serif);
        };
        player.collisionDetected(area).then(g);
    }

    override float radius() {
        return 1;
    }

    override vec3 center() {
        return pos;
    }

    private void updateGeometry() {
        //import std.algorithm : map, sum;
        //auto vs = geom.attributeList;
        //foreach (ref v; vs) {
        //    v.normal = vec3(0);
        //}
        //foreach (i; 0..geom.indexList.length/3) {
        //    auto v0 = vs[geom.indexList[i*3+0]];
        //    auto v1 = vs[geom.indexList[i*3+1]];
        //    auto v2 = vs[geom.indexList[i*3+2]];
        //    auto normal = normalize(cross(
        //        v2.position.xyz - v0.position.xyz,
        //        v1.position.xyz - v0.position.xyz));
        //    v0.normal += normal;
        //    v1.normal += normal;
        //    v2.normal += normal;
        //}
        //foreach (i,ref v; vs) {
        //    auto p = particleList[i];
        //    v.position = p.position;
        //    v.normal = safeNormalize(p.normal);
        //}
        //geom.update();
    }
}

class NPCMaterial : Material {
    mixin PatchVertices!(3);

    mixin VertexShaderSource!(q{
        #version 450
        in vec4 position;
        in vec3 normal;
        out vec3 normal2;

        void main() {
            gl_Position = position;
            normal2 = normal;
        }
    });

    mixin TessellationControlShaderSource!q{
        #version 450
        layout(vertices=3) out;
        in vec3 normal2[];
        out vec3 normal3[];
        uniform int outerLevel;
        uniform int innerLevel;

        void main(){
            gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
            normal3[gl_InvocationID] = normal2[gl_InvocationID];
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
        out vec3 normal4;

        void main(){
            vec4 p = vec4(0);
            vec3 n = vec3(0);
            for (int i = 0; i < 3; i++) {
                p += gl_TessCoord[i] * gl_in[i].gl_Position;
                n += gl_TessCoord[i] * normal3[i];
            }

            gl_Position = p;
            normal4 = n;
        }
    };

    mixin GeometryShaderSource!q{
        #version 450
        layout(triangles) in;
        layout(triangle_strip,max_vertices=4) out;
        in vec3 normal4[];
        out vec3 pos;
        out vec3 n;
        uniform mat4 worldMatrix;
        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;
        
        void main(){
            for (int i = 0; i < 3; i++) {
                vec4 p = gl_in[i].gl_Position;
                gl_Position = projectionMatrix * viewMatrix * worldMatrix * p;
                pos = (viewMatrix * worldMatrix * p).xyz;
                n = (viewMatrix * worldMatrix * vec4(normal4[i],0)).xyz;
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
            vec3 col = vec3(0.8,0.2,0.1);
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
