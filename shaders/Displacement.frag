#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = normalize(cross(n, t));

  mat3 TBN = mat3(t, b, n);

  vec2 uv = v_uv;

  float du = 1.0 / u_texture_2_size.x;
  float dv = 1.0 / u_texture_2_size.y;

  float h0 = h(uv);
  float hU = h(uv + vec2(du, 0.0));
  float hV = h(uv + vec2(0.0, dv));

  float dU = (hU - h0) * u_height_scaling * u_normal_scaling;
  float dV = (hV - h0) * u_height_scaling * u_normal_scaling;

  vec3 local_normal = normalize(vec3(-dU, -dV, 1.0));
  vec3 displaced_normal = normalize(TBN * local_normal);

  vec3 p = v_position.xyz;
  vec3 l = normalize(u_light_pos - p);
  float r2 = dot(u_light_pos - p, u_light_pos - p);

  vec3 color = u_color.rgb *
               (u_light_intensity / r2) *
               max(0.0, dot(displaced_normal, l));

  out_color = vec4(color, 1.0);
}

