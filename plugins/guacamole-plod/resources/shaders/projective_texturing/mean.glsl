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

layout(r32f) uniform writeonly image2D mean_buffer;
layout(local_size_x = 1, local_size_y = 1) in;

uniform sampler2D input_buffer;

void main() {
  ivec2 store_pos = ivec2(gl_GlobalInvocationID.xy);
  vec2 resolution = gl_NumWorkGroups.xy;

  vec4 color = texelFetch(input_buffer, store_pos, 0);

  float normalized_color = (color.r + color.g + color.b)/3.0;
  normalized_color /= resolution.x * resolution.y;

  imageStore(mean_buffer, store_pos, vec4(normalized_color));
}

