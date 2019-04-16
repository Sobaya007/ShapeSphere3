import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import root;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {

    auto camera = proj.get!Camera("camera");
    auto canvas = proj.get!Canvas("canvas");
    auto window = proj.get!Window("window");
    auto sphere = new ElasticSphere;
    proj["sphere"] = sphere;
    with (context()) {
        import std.datetime;
        import std.conv : to;

        auto before = Clock.currTime;
        when(Frame).then({
            auto now = Clock.currTime;
            window.title = (now - before).total!"msecs".to!string;
            before = now;

            auto floor = proj.get!Floor("floor");
            sphere.step(floor);
            camera.capture(sphere);
        });

        float pushCount = 0;
        when(KeyButton.Space.pressing).then({
            enum DOWN_PUSH_FORCE = 600;
            enum DOWN_PUSH_FORE_MIN = 800;
            pushCount += 0.1;
            sphere.push(vec3(0,-1,0) * DOWN_PUSH_FORCE * pushCount, DOWN_PUSH_FORE_MIN);

            if (pushCount > 1) pushCount = 1;
        });
        when(KeyButton.Space.released).then({
            pushCount = 0;
        });
    }
}

class ElasticSphere : Entity, CollisionCapsule {
    import std.traits : ReturnType;
    import std.typecons : Nullable;

    mixin ImplPos;
    mixin ImplRot;
    mixin ImplWorldMatrix;
    mixin Material!(ElasticMaterial);
    mixin ImplUniform;
    mixin ImplAABB;

    private static immutable {
        uint RecursionLevel = 2;
        float DefaultRadius = 0.5;
        float Radius = 2.0f;
        float TimeStep = 0.02;
        float Mass = 0.05;
        float Friction = 0.3;
        float Zeta = 0.5;
        float Omega = 150;
        float Gravity = 100;
        float BaloonCoef = 20000;
        float MaxVelocity = 40;
        int IterationCount = 10;

        float C = 2 * Zeta * Omega * Mass;
        float K = Omega * Omega * Mass;
        float VelCoef = 1 / (1 + TimeStep*C/Mass + TimeStep*TimeStep*K/Mass);
        float PosCoef = - (TimeStep*K/Mass) / (1 + TimeStep*C/Mass + TimeStep*TimeStep*K/Mass);
        float ForceCoef = (TimeStep/Mass) / (1 + TimeStep*C/Mass + TimeStep*TimeStep*K/Mass);
    }

    private {
        alias Geometry = ReturnType!(GeometryLibrary().buildIcosahedron);
        Particle[] particleList;
        Pair[] pairList;
        Geometry geom;
        vec3 beforePos = vec3(0);
        float _radius = Radius;
    }

    public {
        vec3 force;
        Nullable!vec3 contactNormal;
    }

    this() {
        import std.algorithm : map;
        import std.array : array;

        this.geom = GeometryLibrary().buildIcosahedron(RecursionLevel).transform(mat3.scale(vec3(DefaultRadius)));
        this.geometry = geom;

        this.force = vec3(0);
        this.particleList = geom.attributeList.map!(a => new Particle(a.position.xyz)).array;

        this.pairList = createPairList(this.geom, this.particleList);
    }

    private auto createPairList(Geometry geom, Particle[] particleList) {
        Pair[] result;

        alias ID = uint[2];
        ID pairID(uint a,uint b) { return a < b ? [a,b] : [b,a]; }

        ID[] pairIDList;

        foreach (i; 0..geom.indexList.length/3) {
            foreach (j; 0..3) {
                import std.algorithm : canFind;

                auto id = pairID(
                    geom.indexList[i*3+(j+0)%3],
                    geom.indexList[i*3+(j+1)%3]);

                if (pairIDList.canFind(id) is true) continue;
                pairIDList ~= id;

                auto p0 = particleList[id[0]];
                auto p1 = particleList[id[1]];
                result ~= new Pair(p0, p1);
                p0.next ~= p1;
                p1.next ~= p0;
            }
        }

        return result;
    }

    void step(Floor floor) {
        import std.algorithm : map, minElement;

        vec3 g = this.center;

        this.rotateParticles(g);
        this.pos = g;

        //拘束解消
        foreach (pair; this.pairList) {
            pair.initialize();
        }
        foreach (k; 0..IterationCount){
            foreach (pair; this.pairList) {
                pair.solve();
            }
        }
        const baloonForce = this.calcBaloonForce(g);
        this.contactNormal.nullify();

        this._radius = this.particleList.map!(p => length(p.position - g)).minElement;
        
        auto colInfo = detect(floor, this);

        /*
        with (Log()) {
            if (colInfo.isNull is false) {
                writeln(colInfo.pushVector(this));
                writeln(colInfo.type);
            }
        }
        */

        foreach (ref particle; this.particleList) {
            particle.force += particle.normal * baloonForce;
            if (this.contactNormal.isNull) particle.force.y -= Gravity * Mass;
            particle.velocity += particle.force * ForceCoef;
            move(particle);
            if (colInfo.isNull is false)
                collision(particle, colInfo.pushVector(this));
            end(particle);
        }
        this.force.y = 0;
        foreach (p; this.particleList) {
            p.force = this.force;
        }
        this.force = vec3(0);

        updateGeometry();
    }

    void push(vec3 forceVector, float maxPower) {
        import std.algorithm : min;
        import std.math : pow;

        auto force = forceVector.length;
        auto n = forceVector / force;
        vec3 g = center;
        const minv = calcMin(-n);
        const maxv = calcMax(-n);

        foreach (p; this.particleList) {
            //下向きの力
            auto v = p.position - g;
            v -= dot(v, n) * n;
            auto len = v.length;
            auto t = (p.position.dot(-n) - minv) / (maxv - minv);
            float power = force / pow(len + 0.6, 2.5);
            power = min(maxPower, power);
            power *= t;
            p.force += power * n;
        }
    }

    private void rotateParticles(vec3 center) {
        import std.math : approxEqual;

        const radius = this.calcRadius(center);
        auto dif = center - this.pos;
        dif.y = 0;
        auto axis = vec3(0,1,0).cross(dif);
        const len = axis.length;
        if (len.approxEqual(0)) return;
        axis /= len;
        auto angle = rad(dif.length / radius);
        quat rot = quat.axisAngle(axis, angle);
        foreach (p; this.particleList) {
            p.position = rot.rotate(p.position-center) + center;
            p.normal = rot.rotate(p.normal);
        }
    }

    private float volume(vec3 center) {
        import std.math : abs;

        float volume = 0;
        foreach (i; 0..geom.indexList.length/3) {
            auto a = this.particleList[geom.indexList[i*3+0]].position - center;
            auto b = this.particleList[geom.indexList[i*3+1]].position - center;
            auto c = this.particleList[geom.indexList[i*3+2]].position - center;
            volume += mat3(a,b,c).determinant;
        }
        return abs(volume) / 6;
    }

    private float area(vec3 center) {
        float area = 0;
        foreach (i; 0..geom.indexList.length/3) {
            auto a = this.particleList[geom.indexList[i*3+0]].position - center;
            auto b = this.particleList[geom.indexList[i*3+1]].position - center;
            auto c = this.particleList[geom.indexList[i*3+2]].position - center;
            area += length(cross(a - b, a - c));
        }
        return area / 2;
    }

    private float calcRadius(vec3 center) {
        import std.algorithm : map, sum;
        return this.particleList.map!(a => length(a.position - center)).sum / this.particleList.length;
    }

    private vec3 calcVelocity() {
        import std.algorithm : map, sum;
        return this.particleList.map!(a => a.velocity).sum / this.particleList.length;
    }

    float calcMin(vec3 n) {
        import std.algorithm : map, reduce, min;
        return this.particleList.map!(p => p.position.dot(n)).reduce!min;
    }

    float calcMax(vec3 n) {
        import std.algorithm : map, reduce, max;
        return this.particleList.map!(p => p.position.dot(n)).reduce!max;
    }

    private float calcBaloonForce(vec3 center) {
        auto area = this.area(center);
        auto volume = this.volume(center);
        return BaloonCoef * area / (volume * this.particleList.length);
    }

    private void move(Particle particle) {
        if (particle.velocity.length > MaxVelocity) {
            particle.velocity *= MaxVelocity / particle.velocity.length;
        }
        particle.position += particle.velocity * TimeStep;
        //particle.capsule.end = particle.capsule.start;
        //particle.capsule.start = particle.position;
    }

    private void collision(Particle particle, vec3 pushVector) {
        const depth = pushVector.length;
        const n = pushVector / depth;
        if (depth < 0) return;
        const po = particle.velocity - dot(particle.velocity, n) * n;
        particle.velocity -= po * Friction;
        if (this.contactNormal.isNull) this.contactNormal = normalize(n);
        else this.contactNormal += normalize(n);
        if (dot(particle.velocity, n) < 0) {
            particle.velocity -= n * dot(n, particle.velocity) * 1;
        }
        particle.position += n * depth;
        if (this.contactNormal.isNull is false)
            this.contactNormal = normalize(this.contactNormal);
    }

    private void end(Particle particle) {
        particle.force = vec3(0,0,0); //用済み
        //particle.capsule.end = particle.capsule.start;
        //particle.capsule.start = particle.position;
    }

    private void updateGeometry() {
        auto vs = geom.attributeList;
        foreach (ref v; vs) {
            v.normal = vec3(0);
        }
        foreach (i; 0..geom.indexList.length/3) {
            auto v0 = vs[geom.indexList[i*3+0]];
            auto v1 = vs[geom.indexList[i*3+1]];
            auto v2 = vs[geom.indexList[i*3+2]];
            auto normal = normalize(cross(
                v2.position.xyz - v0.position.xyz,
                v1.position.xyz - v0.position.xyz));
            v0.normal += normal;
            v1.normal += normal;
            v2.normal += normal;
        }
        foreach (i,ref v; vs) {
            auto p = this.particleList[i];
            v.normal = safeNormalize(p.normal);
            v.position.xyz = (this.worldMatrix.invert() * vec4(p.position, 1)).xyz; }
        geom.update();
    }

    //山登りで探索
    public Particle getNearestParticle(vec3 pos) {
        import std.algorithm : minElement;

        Particle particle = this.particleList[0];
        float minDist = length(particle.position - pos);
        while (true) {
            Particle newParticle = particle.next.minElement!(p => length(p.position - pos));
            float dist = length(newParticle.position - pos);
            if (dist < minDist) {
                minDist = dist;
            } else {
                return particle;
            }
            particle = newParticle;
        }
    }

    private auto center() {
        import std.algorithm : map, sum;

        return this.particleList.map!(p => p.position).sum / this.particleList.length;
    }

    private auto lVel() {
        import std.algorithm : map, sum;

        return this.particleList.map!(p => p.velocity).sum / this.particleList.length;
    }

    private auto aVel() {
        import std.algorithm : map, sum;

        auto c = center;
        auto l = lVel;
        return this.particleList.map!((p) {
            auto r = p.position - center;
            auto v = p.velocity - lVel;
            return cross(r, v) / lengthSq(r);
        }).sum / this.particleList.length;
    }

    override float radius() {
        return _radius;
    }

    override vec3[2] ends() {
        return [pos, beforePos];
    }

    class Particle : CollisionCapsule {
        vec3 position; /* in World, used for Render */
        vec3 velocity;
        vec3 normal; /* in World */
        vec3 force;
        bool isStinger;
        Particle[] next;
        private vec3[2] _ends;

        mixin ImplAABB;

        this(vec3 p) {
            this.position = p;
            this.normal = normalize(p);
            this.velocity = vec3(0);
            this.force = vec3(0,0,0);
            this._ends = [p,p];
        }

        void move() {
           this.force = vec3(0,0,0); //用済み
           this._ends[1] = this._ends[0];
           this._ends[0] = this.position;
        }

        override float radius() {
            return 0.1;
        }

        override vec3[2] ends() {
            return _ends;
        }
    }

    class Pair {
        private Particle p0, p1;
        private vec3 dist;
        private vec3 force;
        private float deflen;

        this(Particle p0, Particle p1) {
            this.p0 = p0;
            this.p1 = p1;
            this.deflen = length(this.p1.position - this.p0.position);
        }

        void initialize() {
            vec3 d = this.p1.position - this.p0.position;
            auto len = d.length;
            if (len > 0) d /= len;
            len -= deflen;
            d *= len;
            this.dist = d;
        }

        void solve() {
            const v1 = this.p1.velocity - this.p0.velocity;
            const v2 = v1 * VelCoef + this.dist * PosCoef;
            const dv = (v2 - v1) * 0.5f;
            this.p0.velocity -= dv;
            this.p1.velocity += dv;
        }
    }
}

class ElasticMaterial : Material {
    mixin VertexShaderSource!q{
        #version 450

        in vec4 position;
        in vec3 normal;
        out vec3 vnormal;
        uniform mat4 worldMatrix;
        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;
        
        void main() {
            gl_Position = projectionMatrix * viewMatrix * worldMatrix * position;
            vnormal = normal;
        }
    };

    mixin FragmentShaderSource!q{
        #version 450

        in vec3 vnormal;
        out vec4 fragColor;
        
        void main() {
            fragColor = vec4(normalize(vnormal) * .5 + .5, 1);
        }
    };
}
