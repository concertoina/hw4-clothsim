#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
  return 0.0;
}

void main() {
  // YOUR CODE HERE
  vec4 displaced_position =
      in_position + vec4(normalize(in_normal.xyz) * h(in_uv) * u_height_scaling, 0.0);

  v_position = u_model * displaced_position;
  v_normal = u_model * vec4(normalize(in_normal.xyz), 0.0);
  v_tangent = u_model * vec4(normalize(in_tangent.xyz), 0.0);
  v_uv = in_uv;

  gl_Position = u_view_projection * v_position;
}
