module project.npc.fukidashi;

import sbylib.graphics;
import sbylib.editor;
import sbylib.collision;
import sbylib.wrapper.freeimage;
import sbylib.wrapper.gl;
import std.datetime.stopwatch : StopWatch;

class Fukidashi : Entity {

    enum LineHeight = 0.1;

    mixin ImplPos;
    mixin ImplRot;
    mixin ImplScale;
    mixin ImplWorldMatrix;
    mixin Material!(FukidashiMaterial);
    mixin ImplUniform;

    private Image image;
    private Entity parent;
    private Angle angle;

    this(Parent)(Project proj, Parent parent, string serif) {
        import std : split, max;

        this.parent = parent;

        this.geometry = GeometryLibrary().buildPlane();

        auto store = GlyphStore("./font/WaonJoyo-R.otf");
        float x = 0, y = 0.6;

        this.image = Image.load("resource/fukidashi.png");
        with (TextureBuilder()) {
            width = image.width;
            height = image.height;
            frameTexture = build(image.dataArray);
        }

        int cnt = 0;
        foreach (lineSerif; serif.split("\\n")) {
            x = max(offsetX(y), offsetX(y + LineHeight));
            foreach (dchar c; lineSerif) {
                auto g = store.getGlyph(c);
                srcPos[cnt] = vec2(g.x, g.y);
                srcSize[cnt] = vec2(g.advance, g.maxHeight);

                dstSize[cnt] = vec2(g.advance, g.maxHeight) * LineHeight / g.maxHeight;
                dstPos[cnt] = vec2(x,y);

                if (outOfBounds(dstPos[cnt] + vec2(dstSize[cnt].x,0)) || outOfBounds(dstPos[cnt] + dstSize[cnt])) {
                    y = dstPos[cnt].y = dstPos[cnt].y - LineHeight;
                    x = dstPos[cnt].x = max(offsetX(y), offsetX(y + LineHeight));
                }

                x += dstSize[cnt].x;
                cnt++;
            }
            y -= LineHeight;
        }
        textTexture = store.texture;
        len = cast(int)serif.length;

        aspect = cast(float)image.width / image.height;

        float t = 0;
        auto camera = proj.get!Camera("camera");
        assert(camera);

        when(KeyButton.Enter.pressed).then({
            t = 0;
        });
        when(this.beforeRender).then({
            this.lookAt(camera.pos);
            t += 0.03;
            if (t < 1) {
                angle = 90.deg * sin(1 * t * 360.deg) / (t * 10);
                enum n = 1.5;
                auto s = 3 * t ^^(2*n) * (3 - 2 * t^^n);
                scale = s;
                auto r = mat3.axisAngle(safeNormalize(camera.pos - this.pos), angle);
                rot = r * rot;
                pos = parent.pos + vec3(0,1.7,0) + r * vec3(0,s*0.1,0);
            }
        });
    }

    private bool outOfBounds(vec2 p) {
        vec2 size = vec2(image.width, image.height) / image.width / 2;
        return lengthSq((p-vec2(0.5)) / size) > 1;
    }

    private float offsetX(float y) {
        import std : sqrt;
        vec2 size = vec2(image.width, image.height) / image.width / 2;
        // (x/w)^^2 + (y/h)^^2 = 1
        y -= 0.5;
        float x = -sqrt(1 - (y/size.y)^^2) * size.x;
        x += 0.5;
        return x;
    }
}

class FukidashiMaterial : Material {

    mixin VertexShaderSource!(q{
        #version 450
        in vec4 position;
        out vec2 uv;
        uniform mat4 worldMatrix;
        uniform mat4 viewMatrix;
        uniform mat4 projectionMatrix;

        void main() {
            gl_Position = projectionMatrix * viewMatrix * worldMatrix * position;
            uv = vec2(-position.x, position.y) + 0.5;
        }
    });

    mixin FragmentShaderSource!(q{
        #version 450
        in vec2 uv;
        out vec4 fragColor;

        uniform vec2 srcPos[64];
        uniform vec2 srcSize[64];
        uniform vec2 dstPos[64];
        uniform vec2 dstSize[64];
        uniform int len;
        uniform float aspect;
        uniform sampler2D textTexture;
        uniform sampler2D frameTexture;

        void main() {
            vec2 uv2 = (uv-vec2(0.5)) * vec2(1,aspect)+ vec2(0.5);
            if (0 <= uv2.x && uv2.x < 1 && 0 <= uv2.y && uv2.y < 1) {
                fragColor = texture2D(frameTexture, uv2).bgra;
            } else {
                discard;
            }
            if (fragColor.a == 0) discard;
            for (int i = 0; i < len; i++) {
                if (dstPos[i].x < uv.x && uv.x < dstPos[i].x + dstSize[i].x
                    && dstPos[i].y < uv.y && uv.y < dstPos[i].y + dstSize[i].y) {
                    vec2 uv2 = (uv - dstPos[i]) / dstSize[i];
                    fragColor.rgb -= texelFetch(textTexture, ivec2(srcPos[i] + vec2(uv2.x, 1-uv2.y) * srcSize[i]), 0).rrr;
                }
            }
        }
    });
}
