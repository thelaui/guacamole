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

// class header
#include <gua/renderer/FrustumVisualizationPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <texture_stream/core/core.hpp>

namespace gua {

FrustumVisualizationPassDescription::FrustumVisualizationPassDescription() : PipelinePassDescription() {
  vertex_shader_ = "resources/shaders/gbuffer/plod/frustum_visualization.vert";
  geometry_shader_ = "resources/shaders/gbuffer/plod/frustum_visualization.geom";
  fragment_shader_ = "resources/shaders/gbuffer/plod/frustum_visualization.frag";
  name_ = "FrustumVisualizationPass";

  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Callback;

  depth_stencil_state_ = boost::make_optional(
    scm::gl::depth_stencil_state_desc(
      true, true, scm::gl::COMPARISON_LESS, true, 1, 0,
      scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    )
  );

  rasterizer_state_ = boost::make_optional(
      scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID,
                                     scm::gl::CULL_NONE,
                                     scm::gl::ORIENT_CCW,
                                     false,
                                     false,
                                     0.0f,
                                     false,
                                     true,
                                     scm::gl::point_raster_state(true)));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> FrustumVisualizationPassDescription::make_copy() const {
  return std::make_shared<FrustumVisualizationPassDescription>(*this);
}

PipelinePass FrustumVisualizationPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};

  auto frustum_vbo_ = ctx.render_device->create_buffer(
      scm::gl::BIND_VERTEX_BUFFER,
      scm::gl::USAGE_DYNAMIC_DRAW,
      1,
      0);

  auto format = scm::gl::vertex_format({
    scm::gl::vertex_format::element(0, 0, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 1, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 2, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 3, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 4, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 5, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 6, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8),
    scm::gl::vertex_format::element(0, 7, scm::gl::TYPE_VEC3D, sizeof(scm::math::vec3d) * 8)
  });

  auto frustum_vao_ = ctx.render_device->create_vertex_array(format, {frustum_vbo_});
  auto frusta_ = texstr::FrustumManagement::fetch_frusta();

  pass.process_ = [frustum_vbo_, frustum_vao_, frusta_](
      PipelinePass &, PipelinePassDescription const&, Pipeline & pipe) {

    RenderContext const& ctx(pipe.get_context());

    if (frusta_.size() > 0) {

      ctx.render_device->resize_buffer(frustum_vbo_, frusta_.size() * 8 * sizeof(scm::math::vec3d));

      {
        auto vbo_mem = static_cast<scm::math::vec3d*>(
                                    ctx.render_context->map_buffer(
                                        frustum_vbo_,
                                        scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

        for (int i(0); i < frusta_.size(); ++i) {
          auto corners(frusta_[i].get_corners());
          for (int c(0); c < corners.size(); ++c) {
            vbo_mem[i * 8 + c] = corners[c];
          }
        }

        ctx.render_context->unmap_buffer(frustum_vbo_);
      }

      ctx.render_context->bind_vertex_array(frustum_vao_);

      ctx.render_context->apply();

      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, frusta_.size());
    }
  };

  return pass;
}

}
