/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

@include "shaders/common/header.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"

uniform uvec2 color_buffer;
uniform uvec2 position_buffer;
uniform uvec2 depth_buffer;

uniform vec2 clipping_parameters;

in vec2 gua_quad_coords;

// write outputs
layout(location=0) out vec3 gua_out_color;

void main() {

  vec3 world_position = texture2D(sampler2D(position_buffer), gua_quad_coords).xyz;
  gua_out_color = vec3(0.0);

  bool fragment_clipped = false;
  if (world_position.y >= clipping_parameters.y) {
    fragment_clipped = true;
  } else {
    vec4 proj_tex_space_pos = gua_projection_matrix * gua_view_matrix * vec4(world_position, 1.0);

    if (proj_tex_space_pos.z >= clipping_parameters.x) {
      fragment_clipped = true;
    }
  }

  if (!fragment_clipped) {
    gua_out_color = texture2D(sampler2D(color_buffer), gua_quad_coords).rgb;
  }
}
