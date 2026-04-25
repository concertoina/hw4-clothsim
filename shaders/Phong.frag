#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  vec3 p = v_position.xyz;
  vec3 n = normalize(v_normal.xyz);

  vec3 l = u_light_pos - p;
  float r2 = dot(l, l);
  vec3 l_dir = normalize(l);

  vec3 v_dir = normalize(u_cam_pos - p);
  vec3 h = normalize(l_dir + v_dir);

  vec3 ka = vec3(0.1);
  vec3 kd = vec3(0.7);
  vec3 ks = vec3(0.4);
  vec3 Ia = vec3(1.0);
  float shininess = 32.0;

  vec3 ambient = ka * Ia;

  vec3 diffuse =
      kd * (u_light_intensity / r2) * max(0.0, dot(n, l_dir));

  vec3 specular =
      ks * (u_light_intensity / r2) *
      pow(max(0.0, dot(n, h)), shininess);

  vec3 color = ambient + diffuse + specular;

  out_color = vec4(color, 1.0);
}

