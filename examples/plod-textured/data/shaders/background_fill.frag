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

mat3 homography = (mat3(
  1.0844150096782323, -0.0216027047911971, -0.0000027239192071,
  0.2705041248015606, 1.1905118212355421, 0.0003194544730919,
  -85.0306662046851187, 3.2737730386715316, 1.0000000000000000
));

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

    vec2 quad_coords = gua_get_quad_coords();

    vec2 pixel_coords = vec2(1280.0 * quad_coords.x,
                              960.0 * (1.0 - (quad_coords.y)));

    vec3 transformed_coord = homography * vec3(pixel_coords, 1.0);
    transformed_coord /= transformed_coord.z;
    transformed_coord.xy /= vec2(1280.0, 960.0);
    transformed_coord.y = 1.0 - transformed_coord.y;

    result = texture(sampler2D(projection_textures[frustum_id]),
                               transformed_coord.xy).rgb;
  }

  return result;
}

void main() {

  if (gua_get_depth() < 1.0 ) {
    discard;
  }

  // gua_out_color = vec3(0.8, 0.8, 1.0);
  gua_out_color = vec3(0.0);

  if (enabled == 1) {
    int frustum_id = get_id_smallest_distance(gua_camera_position_4);
    gua_out_color = get_color(frustum_id);
  }

  // gua_out_color = toneMap(gua_out_color);

}
