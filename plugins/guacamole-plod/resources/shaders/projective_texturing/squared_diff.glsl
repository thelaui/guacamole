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

layout(r32f) uniform writeonly image2D squared_diff_buffer;
layout(local_size_x = 8, local_size_y = 8) in;

// uniform uvec2 color_buffer;
// uniform uvec2 photo;

uniform sampler2D color_buffer;
uniform sampler2D photo;

void main() {
  ivec2 store_pos = ivec2(gl_GlobalInvocationID.xy);
  vec2 resolution = gl_NumWorkGroups.xy * gl_WorkGroupSize.xy;

  // if (photo != uvec2(0) && color_buffer != uvec2(0)) {

    // vec4 rendered_color = texelFetch(sampler2D(color_buffer), store_pos, 0);
    // vec4 photo_color = texture(sampler2D(photo), store_pos / (1.0 * resolution));

    vec4 rendered_color = texelFetch(color_buffer, store_pos, 0);
    vec4 photo_color = texture(photo, store_pos / (1.0 * resolution));

    // float rendered_average = (rendered_color.r + rendered_color.g + rendered_color.b)/3.0;
    // float photo_average = (photo_color.r + photo_color.g + photo_color.b)/3.0;

    // vec4 out_color = vec4(rendered_average);
    vec4 out_color = vec4(rendered_color.r);
    // vec4 out_color = vec4(photo_average);

    if (out_color.r != 0.0) {
      float squared_diff = pow(out_color.r - photo_color.r, 2);
      out_color = vec4(squared_diff);
    }

    imageStore(squared_diff_buffer, store_pos, out_color);
  // }
}

