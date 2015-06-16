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

layout(points) in;
layout(line_strip, max_vertices = 16) out;

in vec4 varying_corner_lbb[];
in vec4 varying_corner_lbf[];
in vec4 varying_corner_ltb[];
in vec4 varying_corner_ltf[];
in vec4 varying_corner_rbb[];
in vec4 varying_corner_rbf[];
in vec4 varying_corner_rtb[];
in vec4 varying_corner_rtf[];
in vec3 varying_color[];

out vec3 frustum_color;

void main() {

  // back quad

  gl_Position = varying_corner_lbb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_ltb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rtb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rbb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  EndPrimitive();

  // bottom quad

  gl_Position = varying_corner_lbf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_lbb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rbb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rbf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  EndPrimitive();

  // front quad

  gl_Position = varying_corner_ltf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_lbf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rbf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rtf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  EndPrimitive();

  // top quad

  gl_Position = varying_corner_ltb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_ltf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rtf[0];
  frustum_color = varying_color[0];
  EmitVertex();

  gl_Position = varying_corner_rtb[0];
  frustum_color = varying_color[0];
  EmitVertex();

  EndPrimitive();

}

