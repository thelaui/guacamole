@include "common/header.glsl"

layout(early_fragment_tests) in;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_global_variable_declaration.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 pass_point_color;
in vec3 pass_normal;
in vec2 pass_uv_coords;
in float pass_log_depth;

@include "common/gua_fragment_shader_input.glsl"

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

@include "common/gua_fragment_shader_output.glsl"

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  vec2 uv_coords = pass_uv_coords;

  //turn normal to viewer
  vec4 view_normal = gua_view_matrix * vec4(pass_normal, 0.0);
  vec3 face_forward_normal = pass_normal.xyz;

  if (view_normal.z < 0.0) {
    face_forward_normal = -face_forward_normal;
  }

  // if( dot(uv_coords, uv_coords) > 1)
  //   discard;

  for (int i=0; i < gua_clipping_plane_count; ++i) {

    if (dot(gua_clipping_planes[i].xyz, gua_varying_world_position.xyz) + gua_clipping_planes[i].w < 0) {
      discard;
    }
  }


  @include "common/gua_global_variable_assignment.glsl"

  gua_normal = pass_normal;
  gua_color = pass_point_color;
  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 1.0;
  gua_alpha      = 1.0;
  gua_flags_passthrough = (gua_emissivity > 0.99999);

  // gl_FragDepth = pass_log_depth;

  @include "common/gua_write_gbuffer.glsl"

}

