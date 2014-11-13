#version 300 es
precision highp float;
uniform float luminance;
uniform sampler2D tex;
layout(location = 0) out vec4 color;
in vec2 uv;

void main() {
	vec4 col_raw = texture(tex, uv);
	color = vec4(luminance * col_raw.xyz, col_raw.w);
}
