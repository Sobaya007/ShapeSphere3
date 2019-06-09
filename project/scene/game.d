module project.scene.game;

import sbylib.graphics;
import sbylib.editor;
import project.player.player;
import project.stage;

mixin(Register!(setupGame));

void setupGame(Project proj, EventContext context) {
    with (context()) {
        setupPlayer(proj, context);
        setupStageRender(proj, context);
        setupStageCollision(proj, context);
        proj["renderScene"] = {
            if (auto renderGameStage = proj.get!(void delegate())("renderGameStage")) {
                renderGameStage();
            }
        };
    }
}
