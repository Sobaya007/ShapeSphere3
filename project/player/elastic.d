module project.player.elastic;

import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.glfw;
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
        import std.algorithm : map, minElement, maxElement, min;
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

        foreach (ref particle; player.particleList) {
            particle.force += particle.normal * baloonForce;
            if (this.contactNormal.isNull) particle.force.y -= GRAVITY * MASS;

            particle.velocity += particle.force * FORCE_COEF;
            particle.velocity = min(MAX_VELOCITY, particle.velocity.length) * particle.velocity.safeNormalize;

            particle.position += particle.velocity * TIME_STEP;
            particle._ends[0] = particle.position;

            foreach (polygon; polygonList) {
                collision(particle, polygon);
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

    private float calcBaloonForce(vec3 center) const {
        auto area = player.calcArea(center);
        auto volume = player.calcVolume(center);
        return BALOON_COEF * area / (volume * player.particleList.length);
    }

    private void collision(Player.Particle particle, ModelPolygon polygon) {
        auto colInfo = detect(polygon, particle);
        if (colInfo.isNull) return;

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
