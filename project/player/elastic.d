module project.player.elastic;

import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import sbylib.wrapper.gl;
import root;
import project.player.player;
import project.stage.collision;
import project.npc.npc;
import std.container : Array;
import std.conv;
import std.datetime.stopwatch : StopWatch;

void setupElastic(Project proj) {

    auto camera = proj.get!Camera("camera");
    auto player = proj.get!Player("player");
    assert(camera);
    assert(player);

    auto context = new EventContext;
    auto elastic = new ElasticBehavior(player, context);
    proj["elastic"] = elastic;

    with (context()) {
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
        ContactSolver contactSolver;
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
        this.detector.positions.data.allocate(500);
        this.detector.normals.data.allocate(500);
        this.detector.result.data.allocate(player.particleList.length * 500);

        /*
        this.contactSolver = new ContactSolver;
        with (contactSolver) {
            polygonPositions.data.allocate(500);
            polygonNormals.data.allocate(500);
            particleIndices.data.allocate(500);
            targetVelocities.data.allocate(500);
            particlePositions.data.allocate(player.particleList.length);
            particleVelocities.data.allocate(player.particleList.length);
        }
        */
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

        foreach (particle; player.particleList) {
            particle.force += particle.normal * baloonForce;
            if (this.contactNormal.isNull) particle.force.y -= GRAVITY * MASS;

            particle.velocity += particle.force * FORCE_COEF;
            particle.velocity = min(MAX_VELOCITY, particle.velocity.length) * particle.velocity.safeNormalize;
            particle._ends[0] = particle._ends[1];
            particle._ends[1] = particle.position;
        }

        player._radius = player.particleList
            .map!(p => player.ends.array.map!(e => length(e - p.position)).minElement).maxElement;
        
        Array!ModelPolygon polygonList;
        stage.polygonSet.detect!(ModelPolygon, Player)(player,
            (ModelPolygon polygon, Player, CapsulePolygonResult) {
            polygonList ~= polygon;
        });
        Array!NPC npcList; 
        stage.polygonSet.detect!(NPC, Player)(player,
            (NPC npc, Player, CapsuleSphereResult) {
            npcList ~= npc;
        });

        calcBroad(polygonList);

        Array!ContactPair contactList;
        foreach (i, ref particle; player.particleList) {
            int cnt;
            foreach (polygon; polygonList) {
                if (detector.result.data[i + cnt * detector.sizeA] == 1) {
                    if (collision(particle, polygon)) {
                        contactList ~= ContactPair(particle, polygon);
                    }
                }
                cnt++;
            }
            foreach (npc; npcList) {
                if (collision(particle, npc)) {
                    contactList ~= ContactPair(particle, npc);
                }
            }
        }

        // 本当はこれでGPGPUしたいが、lockのしかたがわからないのでやめとく
        //solveContact(contactList);

        foreach (i; 0..10) { // 本当は全員が条件を満たすまでwhile(true)が良いが、たまに固まるのでやめとく
        //while (true) {
            foreach (contact; contactList) {
                contact.solve();
            }
            //bool finished = true;
            //foreach (contact; contactList) {
            //    finished &= contact.solved();
            //}
            //if (finished) break;
        }

        //拘束を解消する過程で、次のフレームで衝突するであろうポリゴンが変わる
        //そっちは見ていないので、そちらの拘束はわからない -> 死ぬ
        //各粒子が拘束を解消するポリゴンをどう決定するべきか
        //例えば、粒子の最大速度を決定しておくと小領域にできる -> 遅くなる
        //やっぱり衝突前に解消するのは無理？
        //めり込みを許容し、すりぬけないことを目的にする
        //つまり、次のフレームでdepth > 0になることを気にしないことにする
        //現在+次のフレームを衝突形状にするのではなく、
        //現在+過去のフレームを衝突形状にするとする。
        //現在めりこみが生じているとき、めりこみが改善する方向にしか速度を与えないことにする

        const contactCount = this.calcContactCount(polygonList);

        foreach (particle; player.particleList) {
            particle.position += particle.velocity * TIME_STEP;
            particle.force = vec3(0);
        }

        this.force.y = 0;
        foreach (p; player.particleList) {
            p.force = this.force * (contactCount > 0 ? 2 : 0.1);
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
            //p.position = rot.rotate(p.position-center) + center; //こういうことをすると壁抜けしちゃう
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

        // デバッグ用CPUバージョン
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

    private bool collision(Player.Particle particle, ModelPolygon polygon) {
        const colInfo = detect(polygon, particle);
        if (colInfo.isNull) return false;
        const n = -normalize(cross(polygon.vertices[0] - polygon.vertices[1], polygon.vertices[1] - polygon.vertices[2]));

        const po = particle.velocity - dot(particle.velocity, n) * n;
        particle.velocity -= po * FRICTION;
        if (this.contactNormal.isNull) this.contactNormal = normalize(n);
        else this.contactNormal += normalize(n);

        if (this.contactNormal.isNull is false)
            this.contactNormal = normalize(this.contactNormal);

        return true;
    }

    private bool collision(Player.Particle particle, NPC npc) {
        const colInfo = detect(npc, particle);
        if (colInfo.isNull) return false;
        return true;
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
            v.position = (p.ends[0] + p.ends[1]) / 2; // for hide oscillation
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

    struct ContactPair {
        Player.Particle particle;
        vec3 pos;
        vec3 normal;
        float targetVel;

        enum AcceptableDepth = 0.0001;

        this(Player.Particle particle, ModelPolygon polygon) {
            this.particle = particle;
            pos = polygon.vertices[0];
            normal = -normalize(cross(polygon.vertices[0] - polygon.vertices[1], polygon.vertices[1] - polygon.vertices[2]));

            import std.algorithm : max;
            const depth = dot(normal, polygon.vertices[0] - particle.position) + particle.radius;
            //targetVel = max(-dot(normal, particle.velocity) * 0.2, depth / TIME_STEP * 0.5); //振動しがち
            //targetVel = (depth-AcceptableDepth) / TIME_STEP * 0.1; //振動しがち
            targetVel = max(0,-dot(normal, particle.velocity) * 0.2); //振動しがち
            //targetVel = 0; //重くなりがち
            //targetVel = 1e-3;
        }

        this(Player.Particle particle, CollisionSphere sphere) {
            import std : min, max;
            this.particle = particle;
            //(v, s + tv - p) = 0
            // t|v|^2 + (s-p,v) = 0
            pos = sphere.center;
            normal = normalize(particle.position - pos);

            import std.algorithm : max;
            const depth = dot(normal, pos - particle.position) + particle.radius;
            //targetVel = max(-dot(normal, particle.velocity) * 0.2, depth / TIME_STEP * 0.5); //振動しがち
            //targetVel = (depth-AcceptableDepth) / TIME_STEP * 0.1; //振動しがち
            targetVel = max(0,-dot(normal, particle.velocity) * 0.2); //振動しがち
            //targetVel = 0; //重くなりがち
            //targetVel = 1e-3;
        }

        void solve() {
            const depth = dot(normal, pos - particle.position) + particle.radius;
            if (depth > AcceptableDepth) {
                particle.position += normal * (depth-AcceptableDepth);
            }
            if (dot(particle.velocity, normal) < targetVel)
                particle.velocity += normal * (targetVel - dot(normal, particle.velocity));
        }

        bool solved() {
            const depth = dot(normal, pos - particle.position) + particle.radius;
            return depth < AcceptableDepth + 1e-2 && dot(particle.velocity, normal) >= targetVel - 1e-3;
        }
    }

    // CPU実装と挙動が違ってしまう。
    // 多分同じparticleの計算を排他制御していないことが問題。
    // やり方がわからないため、とりあえず放置。
    private deprecated void solveContact(Array!ContactPair contactPairList) {
        //with (contactSolver) {
        //    particlePositions.data.allocate(player.particleList.length);
        //    particleVelocities.data.allocate(player.particleList.length);
        //    int idx;
        //    foreach (contactPair; contactPairList) {
        //        polygonPositions.data[idx] = contactPair.polygon.vertices[0];
        //        polygonNormals.data[idx] = contactPair.normal;
        //        targetVelocities.data[idx] = contactPair.targetVel;
        //        particleIndices.data[idx] = cast(int)contactPair.particle.index;
        //        idx++;
        //    }
        //    foreach (i, particle; player.particleList) {
        //        particlePositions.data[i] = particle.position;
        //        particleVelocities.data[i] = particle.velocity;
        //        radius = particle.radius;
        //    }
        //    acceptableDepth = ContactPair.AcceptableDepth;

        //    polygonPositions.send();
        //    polygonNormals.send();
        //    targetVelocities.send();
        //    particleIndices.send();
        //    particlePositions.send();
        //    particleVelocities.send();

        //    foreach (i; 0..10) {
        //        compute([cast(int)contactPairList.length, 1, 1]);
        //    }

        //    particlePositions.fetch();
        //    particleVelocities.fetch();

        //    foreach (i, particle; player.particleList) {
        //        particle.position = particlePositions.data[i];
        //        particle.velocity = particleVelocities.data[i];
        //    }
        //}
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

// cf. solveContact
deprecated class ContactSolver : Compute {

    mixin ComputeShaderSource!q{
        #version 450

        layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

        uniform float acceptableDepth;
        uniform float radius;

        layout(std430,binding=0) readonly buffer PolygonPosition {
            vec3 data[];
        } polygonPositions;

        layout(std430,binding=1) readonly buffer PolygonNormals {
            vec3 data[];
        } polygonNormals;

        layout(std430,binding=2) readonly buffer TargetVelocity {
            float data[];
        } targetVelocities;

        layout(std430,binding=3) readonly buffer ParticleIndex {
            int data[];
        } particleIndices;

        layout(std430,binding=4) buffer ParticlePosition {
            vec3 data[];
        } particlePositions;

        layout(std430,binding=5) buffer ParticleVelocity {
            vec3 data[];
        } particleVelocities;

        void main() {
            uint idx = gl_GlobalInvocationID.x;
            int particleIndex = particleIndices.data[idx];
            vec3 polygonP = polygonPositions.data[idx];
            vec3 normal = polygonNormals.data[idx];
            vec3 particleP = particlePositions.data[particleIndex];
            vec3 particleV = particleVelocities.data[particleIndex];
            float targetV = targetVelocities.data[idx];

            float depth = dot(normal, polygonP - particleP) + radius;
            if (depth > acceptableDepth) {
                particlePositions.data[particleIndex] += normal * (depth-acceptableDepth);
            }
            if (dot(particleV, normal) < targetV) {
                particleVelocities.data[particleIndex] += normal * (targetV - dot(normal, particleV));
            }
        }
    };
}
