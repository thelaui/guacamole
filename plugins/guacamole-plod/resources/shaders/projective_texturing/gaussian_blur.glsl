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

layout(r32f) uniform writeonly image2D target_blurred_buffer;
layout(local_size_x = 8, local_size_y = 8) in;

#define KERNEL_SIZE 5

const float gauss_kernel[25] = float[](
  0.003765, 0.015019, 0.023792, 0.015019, 0.003765,
  0.015019, 0.059912, 0.094907, 0.059912, 0.015019,
  0.023792, 0.094907, 0.150342, 0.094907, 0.023792,
  0.015019, 0.059912, 0.094907, 0.059912, 0.015019,
  0.003765, 0.015019, 0.023792, 0.015019, 0.003765
);

// const float gauss_kernel[121] = float[](
//   0,0,0,0,0.000001,0.000001,0.000001,0,0,0,0,
//   0,0,0.000001,0.000014,0.000055,0.000088,0.000055,0.000014,0.000001,0,0,
//   0,0.000001,0.000036,0.000362,0.001445,0.002289,0.001445,0.000362,0.000036,0.000001,0,
//   0,0.000014,0.000362,0.003672,0.014648,0.023204,0.014648,0.003672,0.000362,0.000014,0,
//   0.000001,0.000055,0.001445,0.014648,0.058433,0.092564,0.058433,0.014648,0.001445,0.000055,0.000001,
//   0.000001,0.000088,0.002289,0.023204,0.092564,0.146632,0.092564,0.023204,0.002289,0.000088,0.000001,
//   0.000001,0.000055,0.001445,0.014648,0.058433,0.092564,0.058433,0.014648,0.001445,0.000055,0.000001,
//   0,0.000014,0.000362,0.003672,0.014648,0.023204,0.014648,0.003672,0.000362,0.000014,0,
//   0,0.000001,0.000036,0.000362,0.001445,0.002289,0.001445,0.000362,0.000036,0.000001,0,
//   0,0,0.000001,0.000014,0.000055,0.000088,0.000055,0.000014,0.000001,0,0,
//   0,0,0,0,0.000001,0.000001,0.000001,0,0,0,0
// );

uniform uvec2 input_buffer;
uniform uvec2 input_mask;

void main() {
  ivec2 store_pos = ivec2(gl_GlobalInvocationID.xy);
  vec2 resolution = gl_NumWorkGroups.xy * gl_WorkGroupSize.xy;

  float mask_value = texture(sampler2D(input_mask), store_pos / resolution, 0).r;

  float blurred_value = 0.0;

  if (mask_value != 0.0) {

    for (int y = 0; y < KERNEL_SIZE; ++y) {
      for (int x = 0; x < KERNEL_SIZE; ++x) {

        vec2 lookup_pos = clamp((store_pos + ivec2(x, y) - ivec2(KERNEL_SIZE/2)) / (1.0 * resolution), 0.0, 1.0);
        int kernel_pos = x + y * KERNEL_SIZE;
        vec3 input_color = texture(sampler2D(input_buffer), lookup_pos).rgb;
        float color_average = (input_color.r + input_color.g + input_color.b)/3.0;
        blurred_value += color_average * gauss_kernel[kernel_pos];

      }
    }

  }

  imageStore(target_blurred_buffer, store_pos, vec4(blurred_value));
}

