@include "shaders/common/header.glsl"

@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/gua_resolve_pass_uniforms.glsl"

@include "shaders/common/gua_gbuffer_input.glsl"
@include "shaders/common/gua_tone_mapping.glsl"

// write outputs
layout(location=0) out vec3 gua_out_color;

layout (std430, binding=2) uniform projective_texure_block {
  mat4  projection_view_mats[50];
  vec4  frustum_positions[50];
  uvec2 projection_textures[50];
  int   frustum_count;
};

uniform int enabled;

int get_id_smallest_distance(in vec4 position) {
  float minimal_distance = 99999.9;
  int result = 0;
  for (int i = 0; i < frustum_count; ++i) {
    float distance = length(position - frustum_positions[i]);
    if (distance < minimal_distance) {
      minimal_distance = distance;
      result = i;
    }
  }

  return result;
}

vec3 get_color(int frustum_id) {
  vec3 result = vec3(0.0);

  // check if texture is loaded
  if (projection_textures[frustum_id] != uvec2(0)) {

    result = texture(sampler2D(projection_textures[frustum_id]),
                               gua_get_quad_coords()).rgb;
  }

  return result;
}

void main() {

  if (gua_get_depth() < 1.0 ) {
    discard;
  }

  gua_out_color = vec3(0.8, 0.8, 1.0);

  if (enabled == 1) {
    int frustum_id = get_id_smallest_distance(gua_camera_position_4);
    gua_out_color = get_color(frustum_id);
  }

  // gua_out_color = toneMap(gua_out_color);

}
