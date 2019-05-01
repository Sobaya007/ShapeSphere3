module project.camera.chase;

import sbylib.graphics;
import sbylib.editor;
import sbylib.math;
import project.player.player;

mixin(Register!(entryPoint));

void entryPoint(Project proj, EventContext context) {
    auto control = new ChaseControl(proj);

    with (context()) {
        VoidEvent e;
        e = when(Frame).then({
            auto camera = proj.get!Camera("camera");
            auto player = proj.get!Player("player");
            if (camera is null) return;
            if (player is null) return;
            e.kill();
            with (player()) {
                when(Frame).then({ control.step(); });
                enum RotLeft = KeyButton.KeyQ;
                enum RotRight = KeyButton.KeyE;
                when(RotLeft.pressing).then({ control.turn(-1); });
                when(RotRight.pressing).then({ control.turn(+1); });
            }
        });
    }
}

class ChaseControl {
    private vec2 arrivalXZ = vec2(1,0);
    private Angle arrivalPhi = 0.3.rad;
    private float arrivalRadius = 10;
    private float radius = 1;
    private vec3 target  = vec3(0);
    private Project proj;
    //private CollisionRay ray = new CollisionRay;

    private static immutable {
        float CHASE_MAX_LENGTH = 20;
        float CHASE_TARGET_SPEED_RATE = 0.2;
        float CHASE_TURN_SPEED = 0.05;
        float CHASE_TURN_RATE = 0.1;
        float CHASE_RADIUS_RATE = 0.1;
    }

    this(Project proj) {
        this.proj = proj;
    }

    void step() {
        import std.algorithm : min;

        auto camera = proj.get!Camera("camera");
        auto player = proj.get!Player("player");
        
        if (camera is null) return;
        if (player is null) return;

        auto d = safeNormalize(camera.pos - target);
        auto phi = asin(d.y);
        phi += (arrivalPhi - phi) * CHASE_TURN_RATE;

        vec2 xz = d.xz;

        target += (player.calcCenter() - target) * CHASE_TARGET_SPEED_RATE;

        xz += (arrivalXZ - xz) * CHASE_TURN_RATE;
        xz = safeNormalize(xz);


        const dir = vec3(xz.x*cos(phi), sin(phi), xz.y*cos(phi));

        //ray.start = target;
        //ray.dir = dir;
        //auto colInfo = Game.getMap().mapEntity.rayCast(ray);
        //if (colInfo.isJust) {
        //    auto r = length(colInfo.unwrap().point - target);
        //    this.arrivalRadius = min(r, CHASE_MAX_LENGTH);
        //}
        this.radius += (this.arrivalRadius - this.radius) * CHASE_RADIUS_RATE;
        camera.pos = target + radius * dir;
        camera.lookAt(target);
    }

    void turn(float value) {
        auto c = cos(rad(CHASE_TURN_SPEED * value));
        auto s = sin(rad(CHASE_TURN_SPEED * value));
        arrivalXZ = normalize(mat2(c, -s, s, c) * arrivalXZ);
    }
}
