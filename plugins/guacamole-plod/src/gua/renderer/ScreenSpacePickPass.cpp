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
  rendermode_ = RenderMode::Custom;

  depth_stencil_state_ = boost::make_optional(
    scm::gl::depth_stencil_state_desc(false, false)
  );

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

  #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string shader_source = factory.read_shader_file("resources/shaders/screen_space_picking.vert");
  #else
    std::string shader_source = Resources::lookup_shader("resources/shaders/screen_space_picking.vert");
  #endif

  auto shader_program_ = ctx.render_device->create_program(
    {ctx.render_device->create_shader(scm::gl::STAGE_VERTEX_SHADER, shader_source)},
    scm::gl::interleaved_stream_capture("out_pick_position")("out_pick_normal"),
    true,  // rasterizer discard
    "screen_space_picking_program"
  );

  auto mouse_pos_vbo_ = ctx.render_device->create_buffer(
    scm::gl::BIND_VERTEX_BUFFER,
    scm::gl::USAGE_DYNAMIC_DRAW,
    sizeof(scm::math::vec2f)
  );

  auto format = scm::gl::vertex_format(
    scm::gl::vertex_format::element(0, 0, scm::gl::TYPE_VEC2F, sizeof(scm::math::vec2f))
  );

  auto mouse_pos_vao_ = ctx.render_device->create_vertex_array(format, {mouse_pos_vbo_});

  auto transform_feedback_buffer_(ctx.render_device->create_buffer(
                                    scm::gl::BIND_TRANSFORM_FEEDBACK_BUFFER,
                                    scm::gl::USAGE_STATIC_READ,
                                    2 * sizeof(scm::math::vec3f))
                                 );

  auto transform_feedback_(ctx.render_device->create_transform_feedback(
                            scm::gl::stream_output_setup(transform_feedback_buffer_))
                          );

  auto transform_feedback_vao_(ctx.render_device->create_vertex_array(
                                scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, 2 * sizeof(scm::math::vec3f))  // pos
                                                      (0, 1, scm::gl::TYPE_VEC3F, 2 * sizeof(scm::math::vec3f)), // nrm
                               {transform_feedback_buffer_})
                              );

  auto window_name(window_name_);

  pass.process_ = [mouse_pos_vbo_, mouse_pos_vao_,
                   transform_feedback_, transform_feedback_buffer_,
                   transform_feedback_vao_, shader_program_, window_name](
      PipelinePass&, PipelinePassDescription const&, Pipeline & pipe) {

    // std::cout << "process! " << std::endl;
    RenderContext const& ctx(pipe.get_context());

    if (window_name != "") {
      auto window(WindowDatabase::instance()->lookup(window_name));

      if (window) {

        // scm::gl::context_vertex_input_guard cvg(ctx.render_context);
        // scm::gl::context_state_objects_guard csg(ctx.render_context);
        // scm::gl::context_image_units_guard cig(ctx.render_context);
        // scm::gl::context_texture_units_guard ctg(ctx.render_context);

        // ctx.render_context->reset();

        auto mouse_pos(window->get_mouse_position());
        auto resolution(window->config.get_resolution());
        std::cout << mouse_pos << std::endl;

        {
          auto vbo_mem = static_cast<scm::math::vec2f*>(
                                      ctx.render_context->map_buffer(
                                          mouse_pos_vbo_,
                                          scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

          vbo_mem[0] = mouse_pos / scm::math::vec2f(resolution.x, resolution.y);
          std::cout << vbo_mem[0] << std::endl;

          ctx.render_context->unmap_buffer(mouse_pos_vbo_);
        }

        ctx.render_context->bind_program(shader_program_);

        ctx.render_context->begin_transform_feedback(transform_feedback_,
                                                     scm::gl::PRIMITIVE_POINTS);

        {
          ctx.render_context->bind_vertex_array(mouse_pos_vao_);

          ctx.render_context->apply();

          ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, 0, 1);
        }

        ctx.render_context->end_transform_feedback();

        {
          auto tf_mem = static_cast<scm::math::vec3f*>(
                                      ctx.render_context->map_buffer(
                                          transform_feedback_buffer_,
                                          scm::gl::ACCESS_READ_ONLY));

          std::cout << "position " << tf_mem[0] << std::endl;
          std::cout << "normal "   << tf_mem[1] << std::endl;

          ctx.render_context->unmap_buffer(transform_feedback_buffer_);
        }

        // ctx.render_context->reset();
        // scm::math::vec3f* data;
        // auto success = ctx.render_context->get_buffer_sub_data(
        //                                         transform_feedback_buffer_,
        //                                         0, 2 * sizeof(scm::math::vec3f),
        //                                         data
        //                                        );

        // if (success) {
        //   std::cout << "success" << std::endl;
        //   auto casted(static_cast<scm::math::vec3f*>(data));
        //   std::cout << casted[0] << std::endl;
        //   std::cout << casted[1] << std::endl;
        // }

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
