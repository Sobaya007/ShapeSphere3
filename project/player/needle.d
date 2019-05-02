module project.player.needle;

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
import std.typecons : Nullable;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {
    bool finished;
    when(Frame).then({
        auto camera = proj.get!Camera("camera");
        auto player = proj.get!Player("player");

        if (camera is null) return;
        if (player is null) return;

        auto needle = new NeedleBehavior(player, context);

        proj["needle"] = needle;

        with (context()) {

            with (player()) {
                StopWatch sw;
                sw.start();

                when(Frame).then({
                    if (auto stage = proj.get!StageModel("stage")) {
                        needle.step(stage);
                        Window.getCurrentWindow().title = sw.peek.to!string;
                        sw.reset();
                    }
                });

                enum Forward = KeyButton.Up;
                enum Backward = KeyButton.Down;
                enum Left = KeyButton.Left;
                enum Right = KeyButton.Right;
                enum Needle = KeyButton.KeyX;

                when(Needle.pressing).then({
                    player.needleCount += 0.08;
                    if (player.needleCount > 1) player.needleCount = 1;
                });
                when(Needle.releasing).then({ 
                    if (player.needleCount == 0) {
                        import project.player.elastic : ElasticBehavior;
                        if (auto elastic = proj.get!ElasticBehavior("elastic")) {
                            elastic.bind();
                            needle.unbind();
                            foreach (particle; player.particleList) {
                                particle.velocity = needle.calcVelocity(particle.position);
                            }
                        }
                        return;
                    }
                    player.needleCount -= 0.05;
                    if (player.needleCount < 0) player.needleCount = 0;
                });

                void move(vec2 v) {
                    const n = needle.contactNormal.get(vec3(0,1,0));
                    needle.lVel += mat3.rotFromTo(vec3(0,1,0), n) * camera.rot * vec3(v.x, 0, v.y) * 0.8;
                }

                when(Forward.pressing).then({ move(vec2(0,-1)); });
                when(Backward.pressing).then({ move(vec2(0,+1)); });
                when(Left.pressing).then({ move(vec2(-1,0)); });
                when(Right.pressing).then({ move(vec2(+1,0)); });
            }
        }
        finished = true;
        context.unbind();
    }).until(() => finished);
}

class NeedleBehavior {

    struct Wall {
        float allowedPenetration;
        float normalImpulseMin;
    }

    private static immutable {
        float TIME_STEP = 0.02;
        float FRICTION = 2.0f;
        float MAX_RADIUS = 1.5f;
        float DEFAULT_RADIUS = 1.0f;
        float MIN_RADIUS = 0.7f;
        Wall NORMAL_WALL = Wall(0.05f, 0f);
        Wall SAND_WALL = Wall(0.5f, -0.3f);
        float MAX_PENETRATION = 0.1f;
        float SEPARATION_COEF = 1f / TIME_STEP;
        float RESTITUTION_RATE = 0.3f;
        float MASS = 1.0f;
        float AIR_REGISTANCE = 0.5f;
        float GRAVITY = 30.0f;

        float MASS_INV = 1.0f / MASS;
        float INERTIA_INV = 2.5 / (MASS * MAX_RADIUS * MAX_RADIUS);
        uint RECURSION_LEVEL = 2;
        float MAX_VELOCITY = 20;
    }

    private {
        vec3 lVel;
        vec3 aVel;
        vec3 _lastDirection;
        Nullable!vec3 contactNormal;
        Player player;
    }

    public {
        EventContext context;
        alias context this;
    }

    this(Player player, EventContext context) {
        this.player = player;
        this.context = context;

        when(this.context.bound).then({
            this.initialize();
        });
    }

    void initialize() {
        //this.needleCount = 0;
        this.lVel = player.calcLinearVelocity();
        this.aVel = player.calcAngularVelocity();
        //this._lastDirection = elasticSphere.lastDirection;
    }


    void step(StageModel stage) {
        if (this.contactNormal.isNull) {
            this.lVel.y -= GRAVITY * TIME_STEP;
        }
        this.lVel -= AIR_REGISTANCE * this.lVel * TIME_STEP / MASS;
        if (this.lVel.length > MAX_VELOCITY) this.lVel *= MAX_VELOCITY / this.lVel.length;
        this.collision(stage);
        const d = this.lVel * TIME_STEP;
        player.beforePos = player.pos;
        this.player.pos += d;
        foreach (particle; this.player.particleList) {
            particle.position += d;
        }
        this.rotateParticles();
        updateGeometry();
    }

    private void collision(StageModel stage) {

        Array!Contact contacts;
        const lastContactNormal = this.contactNormal;
        this.contactNormal.nullify();
        Nullable!Contact newWallContact;
        Nullable!Contact lastWallContact;
        stage.polygonSet.detect!(ModelPolygon, Player)(player,
            (ModelPolygon polygon, Player player, CapsulePolygonResult result) {

            auto contact = Contact(result, this);
            contacts ~= contact;
            if (polygon.materialName == "Sand") {
                auto nc = result.pushVector(player).normalize;
                if (this.contactNormal.isNull || nc.y < this.contactNormal.y) {
                    this.contactNormal = nc;
                    if (this.contactNormal != lastContactNormal) {
                        newWallContact = contact;
                    } else {
                        lastWallContact = contact;
                    }
                }
            }
        });
        if (newWallContact.isNull is false) {
            newWallContact.wall = SAND_WALL;
        } else if (lastWallContact.isNull is false) {
            lastWallContact.wall = SAND_WALL;
        }
        foreach (ref c; contacts) {
            c.initialize();
        }
        foreach (i; 0..10) {
            foreach (contact; contacts) {
                contact.solve();
            }
        }
    }

    vec3 calcVelocity(vec3 pos) {
        auto r = pos - this.player.pos;
        return this.lVel + cross(this.aVel, r);
    }

    private float calcRadius() {
        import std.algorithm : map, sum;

        vec3 center = player.pos;
        return player.particleList.map!(a => (a.position - center).length).sum / player.particleList.length;
    }

    private void rotateParticles() {
        quat rot = quat.axisAngle(this.aVel.safeNormalize, this.aVel.length.rad * TIME_STEP);
        foreach (p; player.particleList) {
            p.position = rot.rotate(p.position-player.pos) + player.pos;
            p.normal = rot.rotate(p.normal);
        }
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
            const p = player.particleList[i];
            v.position = p.position;
            v.normal = safeNormalize(p.normal);
        }
        player.geom.update();
        player.center = player.particleList.map!(p => p.position).sum / player.particleList.length;
    }

    class NeedleParticle {
        vec3 position; /* in World, used for Render */
        vec3 normal; /* in World */
        vec3 force;
        bool isGround;
        bool isStinger;
        NeedleParticle[] next;

        this(vec3 p) {
            this.position = p;
            this.normal = normalize(p);
            this.force = vec3(0,0,0);
        }

        void move() {
           //this.particle.normal = normalize(this.position - entity.obj.pos);
            //this.position = this.normal * this.getLength(this.isStinger);
           //this.force = vec3(0,0,0); //用済み
        }

        /*
        private float getLength(bool isNeedle){
            alias t = needleCount;
            auto maxLength = isNeedle ? MAX_RADIUS : MIN_RADIUS;
            return t * (maxLength - DEFAULT_RADIUS) + DEFAULT_RADIUS;
        }
        */
    }

    struct Contact {
        NeedleBehavior sphere;
        vec3 normal, tan, bin;
        vec3 nTorque;
        vec3 tTorque;
        vec3 bTorque;
        vec3 nTorqueUnit;
        vec3 tTorqueUnit;
        vec3 bTorqueUnit;
        float normalDenominator;
        float tanDenominator;
        float binDenominator;
        float targetNormalLinearVelocity;
        float normalTotalImpulse;
        float tanTotalImpulse;
        float binTotalImpulse;
        CapsulePolygonResult info;
        Wall wall;

        this(CapsulePolygonResult info, NeedleBehavior sphere) {
            this.info = info;
            this.sphere = sphere;
            this.wall = NORMAL_WALL;
        }

        void initialize() {
            import std.algorithm : min;

            const pushVector = info.pushVector(sphere.player);
            this.normal = pushVector.safeNormalize();
            this.normalTotalImpulse = 0;
            this.tanTotalImpulse = 0;
            this.binTotalImpulse = 0;

            //tangentベクトルは物体間の相対速度から法線成分を抜いた方向
            const center = sphere.player.pos;
            auto colPoint = center - MAX_RADIUS * this.normal;
            const colPointVel = sphere.calcVelocity(colPoint);
            auto relativeColPointVel = colPointVel;
            auto relativeColPointVelNormal = dot(relativeColPointVel, normal);
            this.tan = normalize(relativeColPointVel - relativeColPointVelNormal * normal);
            if (tan.hasNaN) { //normalizeに失敗したら適当にnormalと垂直になるように作る
                tan = vec3(normal.y * normal.x - normal.z * normal.z,
                         -normal.z * normal.y - normal.x * normal.x,
                        normal.x * normal.z + normal.y * normal.y)
                .normalize;
            }

            assert(tan.hasNaN == false);
            //binはnormal・tanと垂直
            this.bin = cross(normal, tan).normalize;

            auto colPointFrom = colPoint - center;
            nTorque = cross(colPointFrom, normal);
            tTorque = cross(colPointFrom, tan);
            bTorque = cross(colPointFrom, bin);
            nTorqueUnit = INERTIA_INV * nTorque;
            tTorqueUnit = INERTIA_INV * tTorque;
            bTorqueUnit = INERTIA_INV * bTorque;

            alias calcDenom = (vec, torqueUnit) =>
                1 / (MASS_INV + dot(vec, cross(torqueUnit, colPointFrom)));
            normalDenominator = calcDenom(normal, nTorqueUnit);
            tanDenominator    = calcDenom(tan,    tTorqueUnit);
            binDenominator    = calcDenom(bin,    bTorqueUnit);

            //if (nvel > -0.02) {
            //    nvel = 0;
            //}

            auto penetration = pushVector.length - wall.allowedPenetration; //ちょっと甘くする
            penetration = min(penetration, MAX_PENETRATION); //大きく埋まりすぎていると吹っ飛ぶ
            const separationVelocity = penetration * SEPARATION_COEF;
            const restitutionVelocity = 0;//-RESTITUTION_RATE * relativeColPointVelNormal;
            this.targetNormalLinearVelocity = separationVelocity;//max(0, separationVelocity);//max(restitutionVelocity, separationVelocity);
        }

        void solve() {
            import std.algorithm : max;

            alias calcColPointVel = (vec, torque) =>
                dot(this.sphere.lVel, vec) + dot(torque, this.sphere.aVel);

            //法線方向の速度についての拘束
            //衝突点における法線方向の相対速度をtargetnormallinearvelocityにする
            //ただし 法線方向撃力 > 0
            //物体間に働く力は必ず斥力だから。
            const colPointVelNormal = calcColPointVel(normal, nTorque);
            const oldNormalImpulse = normalTotalImpulse;
            auto newNormalImpulse = (targetNormalLinearVelocity - colPointVelNormal) * normalDenominator;

            this.normalTotalImpulse += newNormalImpulse;
            auto borderNormalImpulse = wall.normalImpulseMin;
            normalTotalImpulse = max(normalTotalImpulse, borderNormalImpulse);
            newNormalImpulse = normalTotalImpulse - oldNormalImpulse; //補正

            const normalImpulseVector = normal * newNormalImpulse;
            sphere.lVel += MASS_INV * normalImpulseVector;
            sphere.aVel += nTorqueUnit * newNormalImpulse;

            //摩擦力を与える
            //摩擦力は基本的に相対速度を0にする撃力
            //ただし摩擦力 < friction * 法線方向の撃力
            const colPointVelTan = calcColPointVel(tan, tTorque);
            const oldTanImpulse  = tanTotalImpulse;
            auto newTanImpulse  = -colPointVelTan * tanDenominator;
            this.tanTotalImpulse += newTanImpulse;

            const colPointVelBin = calcColPointVel(bin, bTorque);
            const oldBinImpulse  = binTotalImpulse;
            auto newBinImpulse  = -colPointVelBin * binDenominator;
            this.binTotalImpulse += newBinImpulse;

            /*
            const maxFrictionSq     = abs(FRICTION * normalTotalImpulse) ^^ 2;
            const currentFrictionSq = tanTotalImpulse^^2 + binTotalImpulse^^2;
            if (maxFrictionSq < currentFrictionSq) { //条件
                auto scale = sqrt(maxFrictionSq / currentFrictionSq);
                this.tanTotalImpulse *= scale;
                this.binTotalImpulse *= scale;
            }
            */

            newTanImpulse = tanTotalImpulse - oldTanImpulse; //補正
            newBinImpulse = binTotalImpulse - oldBinImpulse; //補正

            const frictionImpulseVector = tan * newTanImpulse + bin * newBinImpulse;
            sphere.lVel += MASS_INV * frictionImpulseVector;
            sphere.aVel += tTorqueUnit * newTanImpulse + bTorqueUnit * newBinImpulse;
        }

        bool hasFinished() {
            import std.math : abs;

            alias calcColPointVel = (vec, torque) =>
                dot(sphere.lVel, vec) + dot(torque, sphere.aVel);
            return abs(calcColPointVel(normal, nTorque) - targetNormalLinearVelocity) < 1e-2;
        }
    }
}
