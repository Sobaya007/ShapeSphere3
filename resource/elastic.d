import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
import root;
import player;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {

    auto window = proj.get!Window("window");
    auto player = proj.get!Player("player");
    auto sphere = new ElasticSphere(player);
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

class ElasticSphere {

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
        Pair[] pairList;
        Player player;
    }

    public {
        import std.typecons : Nullable;

        vec3 force;
        Nullable!vec3 contactNormal;
    }

    this(Player player) {
        this.player = player;
        this.force = vec3(0);
        this.pairList = createPairList(player.geom.indexList, player.particleList);
    }

    private auto createPairList(uint[] indexList, Player.Particle[] particleList) {
        Pair[] result;

        alias ID = uint[2];
        ID pairID(uint a,uint b) { return a < b ? [a,b] : [b,a]; }

        ID[] pairIDList;

        foreach (i; 0..indexList.length/3) {
            foreach (j; 0..3) {
                import std.algorithm : canFind;

                auto id = pairID(
                    indexList[i*3+(j+0)%3],
                    indexList[i*3+(j+1)%3]);

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

    void step(Floor floor) {
        import std.algorithm : map, maxElement, min;

        const g = player.calcCenter();

        this.rotateParticles(g);
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

        player._radius = player.particleList.map!(p => length(p.position - g)).maxElement;
        
        const colInfo = detect(floor, player);

        foreach (ref particle; player.particleList) {
            particle.force += particle.normal * baloonForce;
            if (this.contactNormal.isNull) particle.force.y -= GRAVITY * MASS;

            particle.velocity += particle.force * FORCE_COEF;
            particle.velocity = min(MAX_VELOCITY, particle.velocity.length) * particle.velocity.safeNormalize;

            particle.position += particle.velocity * TIME_STEP;
            particle._ends[0] = particle.position;

            if (colInfo.isNull is false)
                collision(particle, floor);

            particle.force = vec3(0);
            particle._ends[1] = particle.position;
        }
        this.force.y = 0;
        foreach (p; player.particleList) {
            p.force = this.force;
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

    private float calcBaloonForce(vec3 center) const {
        auto area = player.calcArea(center);
        auto volume = player.calcVolume(center);
        return BALOON_COEF * area / (volume * player.particleList.length);
    }

    private void collision(Player.Particle particle, Floor floor) {
        auto colInfo = detect(floor, particle);
        if (colInfo.isNull) return;

        const pushVector = colInfo.pushVector(particle);
        const depth = pushVector.length;
        const n = pushVector / depth;
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
    }

    private void updateGeometry() {
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
            v.position = vec4(p.position, 1);
            v.normal = safeNormalize(p.normal);
        }
        player.geom.update();
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
