@include "shaders/common/header.glsl"

@include "shaders/common/gua_camera_uniforms.glsl"

@include "shaders/common/gua_gbuffer_input.glsl"

// write outputs
layout(location=0) out vec3 gua_out_color;

layout (binding=2) uniform projective_texure_block {
  mat4  projection_view_mats[64];
  vec4  frustum_positions[64];
  uvec4 optimization_states[64];
  uvec2 projection_textures[64];
  int   frustum_count;
  int   pad;
};

uniform int selection_mode;
uniform float blending_factor;
uniform int clipping_enabled;
uniform vec2 clipping_params; // x: depth, y: height
uniform int lens_enabled;
uniform vec4 pick_pos_and_radius;
uniform vec3 pick_normal;
uniform int show_optimized;

vec3 get_color_by_optimization_status(float status) {
  if (status == 0.0) {
    // non-optimized
    return vec3(170.f, 59.f, 56.f) / vec3(255.0, 255.0, 255.0);
  } else if (status == 1.0) {
    // optimized
    return vec3(121.f, 158.f, 52.f) / vec3(255.0, 255.0, 255.0);
  }

  return vec3(1.0);
}

int get_id_smallest_distance(in vec4 position) {
  float minimal_distance = 99999.9;
  int result = 0;
  for (int i = 0; i < frustum_count; ++i) {
    float current_distance = length(position - frustum_positions[i]);
    if (current_distance < minimal_distance) {
      minimal_distance = current_distance;
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
      float current_distance = length(position - frustum_positions[i]);
      if (current_distance < minimal_distance) {
        vec4 proj_tex_space_pos = projection_view_mats[i] * position;

        float depth = proj_tex_space_pos.z;
        // perspective division
        proj_tex_space_pos /= proj_tex_space_pos.w;

        if (abs(proj_tex_space_pos.x) <  1.0 &&
            abs(proj_tex_space_pos.y) <  1.0 &&
            depth                     >= 0.0) {

          minimal_distance = current_distance;
          result = i;
        }
      }
    }
  }

  return result;
}

vec3 get_projected_color(int frustum_id) {
  vec3 result = vec3(0.0);

  // check if texture is loaded
  if (projection_textures[frustum_id] != uvec2(0)) {

    // project fragment position into the projective texture
    vec4 proj_tex_space_pos = projection_view_mats[frustum_id] * vec4(gua_get_position(), 1.0);

    float depth = proj_tex_space_pos.z;
    // perspective division
    proj_tex_space_pos /= proj_tex_space_pos.w;

    // check if fragment is visible by frustum
    if (abs(proj_tex_space_pos.x) <  1.0 &&
        abs(proj_tex_space_pos.y) <  1.0 &&
        depth                     >= 0.0) {



      result = texture(sampler2D(projection_textures[frustum_id]), proj_tex_space_pos.xy * 0.5 + 0.5).rgb;
    }
  }

  return result;
}

vec3 get_projected_color_with_current_camera(int frustum_id) {
  vec3 result = vec3(0.0);

  // check if texture is loaded
  if (projection_textures[frustum_id] != uvec2(0)) {

    // project fragment position into the projective texture
    vec4 proj_tex_space_pos = gua_projection_matrix * gua_view_matrix * vec4(gua_get_position(), 1.0);

    float depth = proj_tex_space_pos.z;
    // perspective division
    proj_tex_space_pos /= proj_tex_space_pos.w;

    // check if fragment is visible by frustum
    if (abs(proj_tex_space_pos.x) <  1.0 &&
        abs(proj_tex_space_pos.y) <  1.0 &&
        depth                     >= 0.0) {

      result = texture(sampler2D(projection_textures[frustum_id]), proj_tex_space_pos.xy * 0.5 + 0.5).rgb;
    }
  }

  return result;
}

float gamma        = 0.80;
float intensity_max = 255.0;

float get_distance_to_plane(in vec3 plane_point, in vec3 plane_normal,
                            in vec3 query_point) {

  return dot(plane_normal, (query_point - plane_point));

}

float round(float d) {
  return floor(d + 0.5);
}

float adjust(float color, float factor) {
  if (color == 0.0){
    return 0.0;
  }
  else{
    float res = round(intensity_max * pow(color * factor, gamma));
    return min(255.0, max(0.0, res));
  }
}

vec3 wavelength_to_rgb(float wavelength) {
  float red, green, blue;
  float factor;

  if(380.0 <= wavelength && wavelength <= 440.0){
    red   = -(wavelength - 440.0) / (440.0 - 380.0);
    green = 0.0;
    blue  = 1.0;
  }
  else if(440.0 < wavelength && wavelength <= 490.0){
    red   = 0.0;
    green = (wavelength - 440.0) / (490.0 - 440.0);
    blue  = 1.0;
  }
  else if(490.0 < wavelength && wavelength <= 510.0){
    red   = 0.0;
    green = 1.0;
    blue  = -(wavelength - 510.0) / (510.0 - 490.0);
  }
  else if(510.0 < wavelength && wavelength <= 580.0){
    red   = (wavelength - 510.0) / (580.0 - 510.0);
    green = 1.0;
    blue  = 0.0;
  }
  else if(580.0 < wavelength && wavelength <= 645.0){
    red   = 1.0;
    green = -(wavelength - 645.0) / (645.0 - 580.0);
    blue  = 0.0;
  }
  else if(645.0 < wavelength && wavelength <= 780.0){
    red   = 1.0;
    green = 0.0;
    blue  = 0.0;
  }
  else{
    red   = 0.0;
    green = 0.0;
    blue  = 0.0;
  }


  if(380.0 <= wavelength && wavelength <= 420.0){
    factor = 0.3 + 0.7*(wavelength - 380.0) / (420.0 - 380.0);
  }
  else if(420.0 < wavelength && wavelength <= 701.0){
    factor = 1.0;
  }
  else if(701.0 < wavelength && wavelength <= 780.0){
    factor = 0.3 + 0.7*(780.0 - wavelength) / (780.0 - 701.0);
  }
  else{
    factor = 0.0;
  }
  float r = adjust(red,   factor);
  float g = adjust(green, factor);
  float b = adjust(blue,  factor);
  return vec3(r/255.0,g/255.0,b/255.0);
}

float get_wave_length_from_data_point(float value, float min_value, float max_value) {
  float min_visible_wave_length = 380.0; //350.0;
  float max_visible_wave_length = 780.0; //650.0;
  //Convert data value in the range of min_values..max_values to the
  //range 350..780
  return (value - min_value) / (max_value-min_value) *
         (max_visible_wave_length - min_visible_wave_length) +
         min_visible_wave_length;
}

void apply_lens() {

  vec3 lens_color = vec3(0.0);
  float radial_fade = 0.0;

  if (lens_enabled == 1) {
    float distance_to_pick = length(pick_pos_and_radius.xyz - gua_get_position().xyz);

    if (distance_to_pick < pick_pos_and_radius.w) {
      radial_fade = 1.0 - pow(distance_to_pick / pick_pos_and_radius.w, 3);
      // radial_fade = 1.0;

      vec3 binormal = vec3(pick_normal.y, -pick_normal.x, 0.0);
      binormal = normalize(binormal);

      float d = dot(binormal, gua_get_normal());

      lens_color = wavelength_to_rgb(get_wave_length_from_data_point(d, -1.0, 1.0));
    }
  }

  gua_out_color = mix(gua_out_color, lens_color, radial_fade);

}

void main() {

  gua_out_color = gua_get_color();

  if (gua_get_depth() < 1.0 ) {

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
          gua_out_color = mix(gua_out_color, projected_color, blending_factor);

          if (show_optimized == 1) {
            float optimization_state = optimization_states[frustum_id].x;
            vec3 optimization_color = get_color_by_optimization_status(optimization_state);
            gua_out_color = mix(gua_out_color, optimization_color, 0.5);
          }
        }
      }

      apply_lens();
    }
  }


}
