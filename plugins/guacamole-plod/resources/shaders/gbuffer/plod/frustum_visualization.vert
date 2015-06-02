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


@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributes
layout(location = 0) in vec3 in_corner_lbb;
layout(location = 1) in vec3 in_corner_lbf;
layout(location = 2) in vec3 in_corner_ltb;
layout(location = 3) in vec3 in_corner_ltf;
layout(location = 4) in vec3 in_corner_rbb;
layout(location = 5) in vec3 in_corner_rbf;
layout(location = 6) in vec3 in_corner_rtb;
layout(location = 7) in vec3 in_corner_rtf;

out vec4 varying_corner_lbb;
out vec4 varying_corner_lbf;
out vec4 varying_corner_ltb;
out vec4 varying_corner_ltf;
out vec4 varying_corner_rbb;
out vec4 varying_corner_rbf;
out vec4 varying_corner_rtb;
out vec4 varying_corner_rtf;

void main() {
  mat4 mat = gua_projection_matrix * gua_view_matrix;

  varying_corner_lbb = mat * vec4(in_corner_lbb, 1.0);
  varying_corner_lbf = mat * vec4(in_corner_lbf, 1.0);
  varying_corner_ltb = mat * vec4(in_corner_ltb, 1.0);
  varying_corner_ltf = mat * vec4(in_corner_ltf, 1.0);
  varying_corner_rbb = mat * vec4(in_corner_rbb, 1.0);
  varying_corner_rbf = mat * vec4(in_corner_rbf, 1.0);
  varying_corner_rtb = mat * vec4(in_corner_rtb, 1.0);
  varying_corner_rtf = mat * vec4(in_corner_rtf, 1.0);
}


