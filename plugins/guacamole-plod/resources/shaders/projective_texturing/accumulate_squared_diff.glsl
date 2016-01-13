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

layout(r32f) uniform writeonly image2D accumulate_buffer;
layout(local_size_x = 1, local_size_y = 1) in;

uniform sampler2D input_buffer;

void main() {
  ivec2 store_pos = ivec2(gl_GlobalInvocationID.xy);

  vec4 accumulated_color = vec4(0.0);

  for (int x = 0; x < 1; ++x) {
    for (int y = 0; y < 1; ++y) {
      accumulated_color += texelFetch(input_buffer, store_pos * 2 + ivec2(x,y), 0);
    }
  }

  imageStore(accumulate_buffer, store_pos, accumulated_color);
}

