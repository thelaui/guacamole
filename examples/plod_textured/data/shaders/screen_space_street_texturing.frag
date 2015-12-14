@include "shaders/common/header.glsl"

@include "shaders/common/gua_camera_uniforms.glsl"

@include "shaders/common/gua_gbuffer_input.glsl"

// write outputs
layout(location=0) out vec3 gua_out_color;

layout (binding=2) uniform projective_texure_block {
  mat4  projection_view_mats[64];
  mat4  homographies[64];
  vec4  frustum_positions[64];
  vec4  projection_texture_resolutions[64];
  uvec2 projection_textures[64];
  int   frustum_count;
  int   pad;
};

uniform int selection_mode;
uniform float blending_factor;
uniform int clipping_enabled;
uniform vec2 clipping_params; // x: depth, y: height

mat4 homography = (mat4(
  1.0844150096782323, -0.0216027047911971, 0.0, -0.0000027239192071,
  0.2705041248015606, 1.1905118212355421, 0.0, 0.0003194544730919,
  0.0, 0.0, 1.0, 0.0,
  -85.0306662046851187, 3.2737730386715316, 0.0, 1.0000000000000000
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

int get_id_closest_valid_projection(in vec4 position) {
  float minimal_distance = 99999.9;
  int result = 0;
  for (int i = 0; i < frustum_count; ++i) {
    if (projection_textures[i] != uvec2(0)) {
      float distance = length(position - frustum_positions[i]);
      if (distance < minimal_distance) {
        vec4 proj_tex_space_pos = projection_view_mats[i] * position;

        float depth = proj_tex_space_pos.z;
        // perspective division
        proj_tex_space_pos /= proj_tex_space_pos.w;

        if (abs(proj_tex_space_pos.x) <  1.0 &&
            abs(proj_tex_space_pos.y) <  1.0 &&
            depth                     >= 0.0) {

          minimal_distance = distance;
          result = i;
        }
      }
    }
  }

  return result;
}

float get_vector_average(in vec3 vector) {
  return (vector.x + vector.y + vector.z) / 3.0;
}

vec3 get_projected_color(int frustum_id) {
  vec3 result = vec3(0.0);

  // check if texture is loaded
  if (projection_textures[frustum_id] != uvec2(0)) {
    vec2 resolution = vec2(projection_texture_resolutions[frustum_id].x,
                           projection_texture_resolutions[frustum_id].y);

    // project fragment position into the projective texture
    vec4 proj_tex_space_pos = projection_view_mats[frustum_id] * vec4(gua_get_position(), 1.0);

    float depth = proj_tex_space_pos.z;
    // perspective division
    proj_tex_space_pos /= proj_tex_space_pos.w;

    // check if fragment is visible by frustum
    if (abs(proj_tex_space_pos.x) <  1.0 &&
        abs(proj_tex_space_pos.y) <  1.0 &&
        depth                     >= 0.0) {

      vec2 pixel_coords = vec2(resolution.x * (proj_tex_space_pos.x * 0.5 + 0.5),
                               resolution.y * (1.0 - (proj_tex_space_pos.y * 0.5 + 0.5)));
      // vec4 transformed_coord = homography * vec4(pixel_coords, 0.0, 1.0);
      vec4 transformed_coord = homographies[frustum_id] * vec4(pixel_coords, 0.0, 1.0);
      transformed_coord /= transformed_coord.w;
      transformed_coord.xy /= resolution;
      transformed_coord.y = 1.0 - transformed_coord.y;

      result = texture(sampler2D(projection_textures[frustum_id]), transformed_coord.xy).rgb;
      // vec3 original_color = gua_get_color();
      // float closest_distance = abs(get_vector_average(original_color) - get_vector_average(result));
      // for (int x = -3; x <= 3; ++x) {
      //   for (int y = -3; y <= 3; ++y) {
      //     vec3 match = texture(sampler2D(projection_textures[frustum_id]), transformed_coord.xy + vec2(x,y)).rgb;
      //     float new_distance = abs(get_vector_average(original_color) - get_vector_average(match));

      //     if (new_distance < closest_distance) {
      //       closest_distance = new_distance;
      //       result = match;

      //     }
      //   }
      // }
    }
  }

  return result;
}

vec3 get_projected_color_with_current_camera(int frustum_id) {
  vec3 result = vec3(0.0);

  // check if texture is loaded
  if (projection_textures[frustum_id] != uvec2(0)) {
    vec2 resolution = vec2(projection_texture_resolutions[frustum_id].x,
                           projection_texture_resolutions[frustum_id].y);

    // project fragment position into the projective texture
    vec4 proj_tex_space_pos = gua_projection_matrix * gua_view_matrix * vec4(gua_get_position(), 1.0);

    float depth = proj_tex_space_pos.z;
    // perspective division
    proj_tex_space_pos /= proj_tex_space_pos.w;

    // check if fragment is visible by frustum
    if (abs(proj_tex_space_pos.x) <  1.0 &&
        abs(proj_tex_space_pos.y) <  1.0 &&
        depth                     >= 0.0) {

      vec2 pixel_coords = vec2(resolution.x * (proj_tex_space_pos.x * 0.5 + 0.5),
                               resolution.y * (1.0 - (proj_tex_space_pos.y * 0.5 + 0.5)));
      // vec4 transformed_coord = homography * vec4(pixel_coords, 0.0, 1.0);
      vec4 transformed_coord = homographies[frustum_id] * vec4(pixel_coords, 0.0, 1.0);
      transformed_coord /= transformed_coord.w;
      transformed_coord.xy /= resolution;
      transformed_coord.y = 1.0 - transformed_coord.y;

      result = texture(sampler2D(projection_textures[frustum_id]), transformed_coord.xy).rgb;
    }
  }

  return result;
}

void main() {

  if (gua_get_depth() >= 1.0 ) {
    discard;
  }

  bool fragment_clipped = false;
  if (clipping_enabled == 1) {
    if (gua_get_position().y >= clipping_params.y) {
      gua_out_color = vec3(0.0);
      fragment_clipped = true;
    } else {
      vec4 proj_tex_space_pos = gua_projection_matrix * gua_view_matrix * vec4(gua_get_position(), 1.0);

      if (proj_tex_space_pos.z >= clipping_params.x) {
        gua_out_color = vec3(0.0);
        fragment_clipped = true;
      }
    }
  }

  if (!fragment_clipped) {
    if (blending_factor > 0.0) {

      int frustum_id = 0;
      if (selection_mode == 0) {
        frustum_id = get_id_smallest_distance(gua_camera_position_4);
      } else if (selection_mode == 1) {
        frustum_id = get_id_closest_valid_projection(vec4(gua_get_position(), 1.0));
      }

      vec3 projected_color = get_projected_color(frustum_id);
      // vec3 projected_color = get_projected_color_with_current_camera(frustum_id);
      if (projected_color != vec3(0.0)) {
        gua_out_color = mix(gua_get_color(), projected_color, blending_factor);
      } else {
        discard;
      }
    } else {
      discard;
    }
  }

}