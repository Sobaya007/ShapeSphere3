module project.player.elastic;

import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import sbylib.wrapper.gl;
import root;
import project.player.player;
import project.stage.stage;
import std.container : Array;
import std.conv;
import std.datetime.stopwatch : StopWatch;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {

    bool finished;
    when(Frame).then({
        auto camera = proj.get!Camera("camera");
        auto player = proj.get!Player("player");

        if (camera is null) return;
        if (player is null) return;

        auto elastic = new ElasticBehavior(player, context);
        proj["elastic"] = elastic;

        with (context()) {

            with (player()) {
                StopWatch sw;
                sw.start();

                when(Frame).then({
                    if (auto stage = proj.get!StageModel("stage")) {
                        elastic.step(stage);
                        Window.getCurrentWindow().title = sw.peek.to!string;
                        sw.reset();
                    }
                });

                enum Down = KeyButton.Space;
                enum Forward = KeyButton.Up;
                enum Backward = KeyButton.Down;
                enum Left = KeyButton.Left;
                enum Right = KeyButton.Right;
                enum Needle = KeyButton.KeyX;
                enum DOWN_PUSH_FORCE = 600;
                enum DOWN_PUSH_FORE_MIN = 800;
                float pushCount = 0;

                when(Down.pressing).then({
                    pushCount += 0.1;
                    elastic.push(vec3(0,-1,0) * DOWN_PUSH_FORCE * pushCount, DOWN_PUSH_FORE_MIN);

                    if (pushCount > 1) pushCount = 1;
                });
                when(Down.released).then({ pushCount = 0; });

                void move(vec2 v) {
                    const n = elastic.contactNormal.get(vec3(0,1,0));
                    elastic.force += mat3.rotFromTo(vec3(0,1,0), n) * camera.rot * vec3(v.x, 0, v.y) * 10;
                }
                when(Forward.pressing).then({ move(vec2(0,-1)); });
                when(Backward.pressing).then({ move(vec2(0,+1)); });
                when(Left.pressing).then({ move(vec2(-1,0)); });
                when(Right.pressing).then({ move(vec2(+1,0)); });
                when(Needle.pressing).then({ 
                    import project.player.needle : NeedleBehavior;
                    if (auto needle = proj.get!NeedleBehavior("needle")) {
                        elastic.unbind();
                        needle.bind();
                    }
                });

                when(context.bound).then({
                    pushCount = 0;
                });
            }
        }
        finished = true;
    }).until(() => finished);
}

class ElasticBehavior {

    private static immutable {
        float TIME_STEP = 0.02;
        float MASS = 0.05;
        float FRICTION = 0.3;
        float ZETA = 0.5;
        float OMEGA = 150;
        float GRAVITY = 100;
        float BALOON_COEF = 20_000;
        float MAX_VELOCITY = 40;
        int ITERATION_COUNT = 10;

        float C = 2 * ZETA * OMEGA * MASS;
        float K = OMEGA * OMEGA * MASS;
        float VEL_COEF = 1 / (1 + TIME_STEP*C/MASS + TIME_STEP*TIME_STEP*K/MASS);
        float POS_COEF = - (TIME_STEP*K/MASS) / (1 + TIME_STEP*C/MASS + TIME_STEP*TIME_STEP*K/MASS);
        float FORCE_COEF = (TIME_STEP/MASS) / (1 + TIME_STEP*C/MASS + TIME_STEP*TIME_STEP*K/MASS);
    }

    private {
        Player player;
        Pair[] pairList;
        AABBCollisionDetection detector;
    }

    public {
        import std.typecons : Nullable;

        vec3 force;
        Nullable!vec3 contactNormal;
        EventContext context;
        alias context this;
    }

    this(Player player, EventContext context) {
        import std.algorithm : map;
        import std.array : array;

        this.player = player;
        this.force = vec3(0);
        this.pairList = player.pairList.map!(pair => new Pair(pair[0], pair[1])).array;

        this.context = context;

        this.detector = new AABBCollisionDetection;
        this.detector.end0s.data.allocate(player.particleList.length);
        this.detector.end1s.data.allocate(player.particleList.length);
        this.detector.positions.data.allocate(100);
        this.detector.normals.data.allocate(100);
        this.detector.result.data.allocate(player.particleList.length * 100);
    }

    void push(const vec3 forceVector, const float maxPower) {
        import std.algorithm : min;
        import std.math : pow;

        const force = forceVector.length;
        const n = forceVector / force;
        const g = player.calcCenter();
        const minv = player.calcMin(-n);
        const maxv = player.calcMax(-n);

        foreach (p; player.particleList) {
            //下向きの力
            auto v = p.position - g;
            v -= dot(v, n) * n;
            const len = v.length;
            const t = (p.position.dot(-n) - minv) / (maxv - minv);
            auto power = force / pow(len + 0.6, 2.5);
            power = min(maxPower, power);
            power *= t;
            p.force += power * n;
        }
    }

    import std.datetime.stopwatch : StopWatch;
    StopWatch sw;

    void step(StageModel stage) {
        import std.algorithm : map, minElement, maxElement, min, max;
        import std.array : array;

        const g = player.calcCenter();

        this.rotateParticles(g);
        player.beforePos = player.pos;
        player.pos = g;

        //拘束解消
        foreach (pair; this.pairList) {
            pair.initialize();
        }
        foreach (k; 0..ITERATION_COUNT){
            foreach (pair; this.pairList) {
                pair.solve();
            }
        }

        const baloonForce = this.calcBaloonForce(g);
        this.contactNormal.nullify();

        player._radius = player.particleList
            .map!(p => player.ends.array.map!(e => length(e - p.position)).minElement).maxElement * 2;
        
        Array!ModelPolygon polygonList;
        stage.polygonSet.detect!(ModelPolygon, Player)(player,
            (ModelPolygon polygon, Player, CapsulePolygonResult result) {
            polygonList ~= polygon;
        });

        sw.reset();

        foreach (particle; player.particleList) {
            particle.force += particle.normal * baloonForce;
            if (this.contactNormal.isNull) particle.force.y -= GRAVITY * MASS;

            particle.velocity += particle.force * FORCE_COEF;
            particle.velocity = min(MAX_VELOCITY, particle.velocity.length) * particle.velocity.safeNormalize;

            particle.position += particle.velocity * TIME_STEP;
            particle._ends[0] = particle.position;
        }

        calcBroad(polygonList);

        int c;
        foreach (i; 0..detector.sizeA * detector.sizeB) {
            c += detector.result.data[i];
        }
        with (Log()) {
            writeln("sizeA = ", detector.sizeA);
            writeln("sizeB = ", detector.sizeB);
            writeln("c = ", c);
            writeln("colCount = ", colCount);
        }
        colCount = 0;

        foreach (i, ref particle; player.particleList) {
            int cnt;
            foreach (polygon; polygonList) {
                if (detector.result.data[i + cnt * detector.sizeA] == 1) {
                    collision(particle, polygon);
                }
                cnt++;
            }

            particle.force = vec3(0);
            particle._ends[1] = particle.position;
        }

        const contactCount = this.calcContactCount(polygonList);
        this.force.y = 0;
        foreach (p; player.particleList) {
            p.force = this.force * (0.1 + contactCount);
        }
        this.force = vec3(0);

        updateGeometry();
    }

    private void rotateParticles(vec3 center) {
        import std.math : approxEqual;

        const radius = player.calcRadius(center);
        auto dif = center - player.pos;
        dif.y = 0;
        auto axis = vec3(0,1,0).cross(dif);
        const len = axis.length;
        if (len.approxEqual(0)) return;
        axis /= len;
        const angle = rad(dif.length / radius);
        const rot = quat.axisAngle(axis, angle);
        foreach (p; player.particleList) {
            p.position = rot.rotate(p.position-center) + center;
            p.normal = rot.rotate(p.normal);
        }
    }

    private void calcBroad(ref Array!ModelPolygon polygonList) {
        with (detector) {
            foreach (i, particle; player.particleList) {
                end0s.data[i] = particle.ends[0];
                end1s.data[i] = particle.ends[1];
                radius = particle.radius;
            }
            end0s.send();
            end1s.send();
            sizeA = cast(int)player.particleList.length;

            int cnt;
            foreach (polygon; polygonList) {
                positions.data[cnt] = polygon.vertices[0];
                normals.data[cnt] =
                    cross(polygon.vertices[0] - polygon.vertices[1], polygon.vertices[1] - polygon.vertices[2]);
                cnt++;
            }
            positions.send();
            normals.send();
            sizeB = cnt;

            compute([sizeA, sizeB, 1]);
            result.fetch();
        }

        /*
        with (detector) {
            sizeA = cast(int)player.particleList.length;
            sizeB = cast(int)polygonList.length;
            foreach (indexA, particle; player.particleList) {
                auto A = particle.getAABB();
                minAs.data[indexA] = A.min;
                maxAs.data[indexA] = A.max;

                int indexB;
                foreach (polygon; polygonList) {
                    auto B = polygon.getAABB();
                    minBs.data[indexB] = B.min;
                    maxBs.data[indexB] = B.max;

                    vec3 minA = minAs.data[indexA];
                    vec3 maxA = maxAs.data[indexA];
                    vec3 minB = minBs.data[indexB];
                    vec3 maxB = maxBs.data[indexB];

                    int r = 1;
                    if (maxA.x < minB.x) r = 0;
                    if (maxA.y < minB.y) r = 0;
                    if (maxA.z < minB.z) r = 0;
                    if (maxB.x < minA.x) r = 0;
                    if (maxB.y < minA.y) r = 0;
                    if (maxB.z < minA.z) r = 0;
                    result.data[indexA + indexB * sizeA] = r;

                    indexB++;
                }
            }
            */
    }


    private float calcBaloonForce(vec3 center) const {
        auto area = player.calcArea(center);
        auto volume = player.calcVolume(center);
        return BALOON_COEF * area / (volume * player.particleList.length);
    }

    int colCount;
    private void collision(Player.Particle particle, ModelPolygon polygon) {
        auto colInfo = detect(polygon, particle);
        if (colInfo.isNull) return;
        colCount++;
        sw.start();

        const n = -normalize(cross(polygon.vertices[0] - polygon.vertices[1], polygon.vertices[1] - polygon.vertices[2]));

        const pushVector = colInfo.pushVector(particle);
        const depth = dot(n, pushVector);

        if (depth < 1e-5) return;

        const po = particle.velocity - dot(particle.velocity, n) * n;
        particle.velocity -= po * FRICTION;
        if (this.contactNormal.isNull) this.contactNormal = normalize(n);
        else this.contactNormal += normalize(n);

        if (dot(particle.velocity, n) < 0) {
            particle.velocity -= n * dot(n, particle.velocity) * 1;
        }
        particle.position += n * depth;
        if (this.contactNormal.isNull is false)
            this.contactNormal = normalize(this.contactNormal);
        sw.stop();
    }

    private size_t calcContactCount(ref Array!ModelPolygon polygonList) {
        import std.algorithm : map, filter, min, sort;
        import std.array : array;
        import std.range : walkLength;

        const contactCount = player.particleList.filter!((particle) {
            foreach (polygon; polygonList) {
                auto info = detect(polygon, particle);
                if (info.isNull) continue;
                if (info.pushVector(particle).length < 0.01) return true;
            }
            return false;
        }).walkLength;

        return contactCount;
    }

    private void updateGeometry() {
        import std.algorithm : map, sum;
        auto vs = player.geom.attributeList;
        foreach (ref v; vs) {
            v.normal = vec3(0);
        }
        foreach (i; 0..player.geom.indexList.length/3) {
            auto v0 = vs[player.geom.indexList[i*3+0]];
            auto v1 = vs[player.geom.indexList[i*3+1]];
            auto v2 = vs[player.geom.indexList[i*3+2]];
            auto normal = normalize(cross(
                v2.position.xyz - v0.position.xyz,
                v1.position.xyz - v0.position.xyz));
            v0.normal += normal;
            v1.normal += normal;
            v2.normal += normal;
        }
        foreach (i,ref v; vs) {
            auto p = player.particleList[i];
            v.position = p.position;
            v.normal = safeNormalize(p.normal);
        }
        player.geom.update();
        player.center = player.particleList.map!(p => p.position).sum / player.particleList.length;
    }

    //山登りで探索
    public Player.Particle getNearestParticle(vec3 pos) {
        import std.algorithm : minElement;

        Player.Particle particle = player.particleList[0];
        float minDist = length(particle.position - pos);
        while (true) {
            Player.Particle newParticle = particle.next.minElement!(p => length(p.position - pos));
            const dist = length(newParticle.position - pos);
            if (dist < minDist) {
                minDist = dist;
            } else {
                return particle;
            }
            particle = newParticle;
        }
    }

    class Pair {
        private Player.Particle p0, p1;
        private vec3 dist;
        private vec3 force;
        private float deflen;

        this(Player.Particle p0, Player.Particle p1) {
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
            const v2 = v1 * VEL_COEF + this.dist * POS_COEF;
            const dv = (v2 - v1) * 0.5f;
            this.p0.velocity -= dv;
            this.p1.velocity += dv;
        }
    }
}

class AABBCollisionDetection : Compute {

    mixin ComputeShaderSource!q{
        #version 450

        layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

        layout(std430,binding=0) readonly buffer End0 {
            vec3 data[];
        } end0s;

        layout(std430,binding=1) readonly buffer End1 {
            vec3 data[];
        } end1s;

        uniform float radius;


        layout(std430,binding=2) readonly buffer Position {
            vec3 data[];
        } positions;

        layout(std430,binding=3) readonly buffer Normal {
            vec3 data[];
        } normals;

        layout(std430,binding=4) writeonly buffer Result {
            uint data[];
        } result;

        uniform int sizeA;
        uniform int sizeB;

        void main() {
            uint indexA = gl_GlobalInvocationID.x % sizeA;
            uint indexB = gl_GlobalInvocationID.x / sizeA;
            vec3 e0 = end0s.data[indexA];
            vec3 e1 = end1s.data[indexA];
            vec3 p = positions.data[indexB];
            vec3 n = normals.data[indexB];

            float s0 = dot(e0 - p, n);
            float s1 = dot(e1 - p, n);
            int r = 1;
            if (s0 * s1 >= 0 && min(abs(s0), abs(s1)) > radius) r = 0;

            result.data[gl_GlobalInvocationID.x] = r;
        }
    };
}
