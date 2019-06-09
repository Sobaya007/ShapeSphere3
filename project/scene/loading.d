module project.scene.loading;

import sbylib.graphics;
import sbylib.editor;

void setupLoading(Project proj, EventContext context) {
    auto canvas = proj.get!Canvas("canvas");
    with (context()) {
        auto text = new Text;
        proj["renderScene"] = {
            with (canvas.getContext()) {
                canvas.color.clear = Color.Black;
                clear(ClearMode.Color, ClearMode.Depth);
                text.render();
            }
        };
    }
}

class Text : Entity {
    mixin ImplPos;
    mixin ImplScale;
    mixin ImplWorldMatrix;
    mixin Material!(TextMaterial);
    mixin ImplUniform;
    private GlyphGeometry geom;

    this() {
        this.geom = new GlyphGeometry("./font/consola.ttf");
        this.geometry = geom;

        float x = 0;
        foreach (i, c; "Now Loading...") {
            auto store = geom.glyphStore;
            const g = store.getGlyph(c);
            const w = cast(float)g.width / g.maxHeight;
            geom.addCharacter(c, vec2(x,0), vec2(w, 1));

            this.xs[i] = x;
            this.ws[i] = w;

            x += cast(float)g.advance / g.maxHeight;
        }

        this.tex = geom.glyphStore.texture;

        when(this.beforeRender).then({
            this.pixelHeight = 50.pixel;
            this.scale.x = this.scale.y;

            const ww = Window.getCurrentWindow().width;
            const wh = Window.getCurrentWindow().height;
            this.pixelX = pixel(ww/2);
            this.pos.x -= x * this.scale.x;
            this.pixelY = pixel(-wh/2+this.pixelHeight);
        });

        int cnt;
        when(Frame).then({
            this.dotCount = 1 + cnt++ / 60 % 3;
        });
    }
}

class TextMaterial : Material {
    mixin VertexShaderSource!(q{
        #version 450
        in vec4 position;
        in vec2 texcoord;
        out vec2 tc;
        out float idx;
        uniform mat4 worldMatrix;
        uniform float xs[14];
        uniform float ws[14];

        void main() {
            gl_Position = worldMatrix * position;
            tc = texcoord;

            idx = -1;
            for (int i = 0; i < 14; i++) {
                if (xs[i] <= position.x && position.x < xs[i] + ws[i]*2) {
                    idx = i;
                }
            }
        }
    });

    mixin FragmentShaderSource!(q{
        #version 450
        in vec2 tc;
        in float idx;
        out vec4 fragColor;
        uniform sampler2D tex;
        uniform int dotCount;

        void main() {
            if (idx < 11+dotCount) {
                fragColor = texelFetch(tex, ivec2(tc), 0).rrrr;
            } else {
                fragColor = vec4(0);
            }
        }
    });
}
