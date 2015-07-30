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
#include <gua/renderer/ScreenSpacePickPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <texture_stream/core/core.hpp>

#include <bitset>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
ScreenSpacePickPassDescription::ScreenSpacePickPassDescription()
  : PipelinePassDescription()
  , window_name_("") {

  vertex_shader_ = "";
  geometry_shader_ = "";
  fragment_shader_ = "";
  name_ = "ScreenSpacePickPass";

  writes_only_color_buffer_ = false;
  enable_for_shadows_ = false;
  rendermode_ = RenderMode::Custom;

}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> ScreenSpacePickPassDescription::make_copy() const {
  return std::make_shared<ScreenSpacePickPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
void ScreenSpacePickPassDescription::set_window_name(std::string const& window_name) {
  window_name_ = window_name;
  touch();
}

////////////////////////////////////////////////////////////////////////////////
std::string const& ScreenSpacePickPassDescription::get_window_name() const{
  return window_name_;
}


////////////////////////////////////////////////////////////////////////////////
PipelinePass ScreenSpacePickPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {
  PipelinePass pass(*this, ctx, substitution_map);

  auto window_name(window_name_);

  pass.process_ = [window_name](
      PipelinePass&, PipelinePassDescription const&, Pipeline & pipe) {

    RenderContext const& ctx(pipe.get_context());

    if (window_name != "") {
      auto window(WindowDatabase::instance()->lookup(window_name));
      auto gbuffer(dynamic_cast<GBuffer*>(pipe.current_viewstate().target));

      if (window && gbuffer) {

        auto mouse_pos(window->get_mouse_position());
        auto window_resolution(window->config.get_resolution());
        auto mouse_pos_ndc(mouse_pos / scm::math::vec2f(window_resolution.x, window_resolution.y));
        auto buffer_resolution(gbuffer->get_resolution());
        auto mouse_pos_buffer(mouse_pos_ndc * scm::math::vec2f(buffer_resolution.x, buffer_resolution.y));

        auto depth_buffer(gbuffer->get_depth_buffer()->get_buffer(ctx));
        auto normal_buffer(gbuffer->get_normal_buffer()->get_buffer(ctx));

        if (depth_buffer && normal_buffer) {
          const int pixel_size(buffer_resolution.x * buffer_resolution.y);
          const int depth_component_size(sizeof(scm::uint32));
          const int normal_component_size(sizeof(unsigned char) * 6);
          scm::uint32 depth_data[pixel_size * depth_component_size];
          unsigned char normal_data[pixel_size * normal_component_size];

          ctx.render_context->retrieve_texture_data(depth_buffer, 0, depth_data);
          // ctx.render_context->retrieve_texture_data(normal_buffer, 0, normal_data);

          auto depth_lookup_pos(int(round(mouse_pos_buffer.x)) +
                                int(round(mouse_pos_buffer.y) * buffer_resolution.x));

          scm::uint32 depth_stencil(depth_data[depth_lookup_pos]);
          scm::uint32 depth_int(depth_stencil & ((1 << 24) - 1));
          double depth(depth_int / double(std::pow(2, 24) - 1.0) * 2.0 - 1.0);

          auto near_clip(pipe.current_viewstate().camera.config.get_near_clip());
          auto far_clip(pipe.current_viewstate().camera.config.get_far_clip());

          auto world_depth(2.0 * near_clip * far_clip /
                           (far_clip + near_clip - depth * (far_clip - near_clip)));


          auto frustum(pipe.current_viewstate().frustum);
          auto projection_view(frustum.get_projection() * frustum.get_view());
          auto world_position(scm::math::inverse(projection_view) *
                              math::vec4(mouse_pos_ndc.x * 2.0 - 1.0,
                                         mouse_pos_ndc.y * 2.0 - 1.0,
                                         -10,
                                         1.0)
                              );

          world_position /= world_position.w;

          std::cout << "depth " << depth << std::endl;
          std::cout << "world_depth " << world_depth << std::endl;
          std::cout << "world_position " << world_position << std::endl;
          std::cout << "frustum position " << frustum.get_camera_position() << std::endl;
          // std::cout << "world_position.x " << world_position.x << std::endl;
          // std::cout << "world_position.y " << world_position.y << std::endl;
          // std::cout << "world_position.z " << world_position.z << std::endl;
          // auto normal(normal_data[depth_lookup_pos]);
          // std::cout << "normal " << normal << std::endl;
        }

      } else {
        Logger::LOG_ERROR << "Couldn't pick: There doesn't exist any window under the name of "
                          << "\"" << window_name << "\"!" << std::endl;
      }

    } else {
      Logger::LOG_ERROR << "Please specify a valid window name!" << std::endl;
    }

  };

  return pass;
}

////////////////////////////////////////////////////////////////////////////////

}
