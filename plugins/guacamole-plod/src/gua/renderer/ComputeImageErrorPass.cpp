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
#include <gua/renderer/ComputeImageErrorPass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <texture_stream/core/core.hpp>

namespace gua {

ComputeImageErrorPassDescription::ComputeImageErrorPassDescription()
  : PipelinePassDescription() {
  name_ = "ComputeImageErrorPass";

  vertex_shader_is_file_name_ = false;
  fragment_shader_is_file_name_ = false;
  geometry_shader_is_file_name_ = false;
  writes_only_color_buffer_ = true;
  needs_color_buffer_as_input_ = true;
  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));
  rendermode_ = RenderMode::Custom;
  uniforms["clipping_parameters"] = gua::math::vec2f(0.0);

}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> ComputeImageErrorPassDescription::make_copy() const {
  return std::make_shared<ComputeImageErrorPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////

void ComputeImageErrorPassDescription::set_clipping_parameters(gua::math::vec2f const& clipping_parameters) {
  clipping_parameters_ = clipping_parameters;
}

////////////////////////////////////////////////////////////////////////////////

gua::math::vec2f const& ComputeImageErrorPassDescription::get_clipping_parameters() const {
  return clipping_parameters_;
}


////////////////////////////////////////////////////////////////////////////////

PipelinePass ComputeImageErrorPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{

  PipelinePass pass(*this, ctx, substitution_map);

  std::string vertex_pass_trough_shader_file("resources/shaders/common/fullscreen_quad.vert");
  std::string clipping_shader_file("resources/shaders/projective_texturing/fragment_clip.frag");
  std::string error_shader_file("resources/shaders/projective_texturing/compute_image_error.glsl");

  #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
      ResourceFactory factory;
      std::string vertex_pass_through_shader_source = factory.read_shader_file(vertex_pass_trough_shader_file);
      std::string clipping_shader_source = factory.read_shader_file(clipping_shader_file);
      std::string error_shader_source = factory.read_shader_file(error_shader_file);
  #else
      std::string vertex_pass_through_shader_source = Resources::lookup_shader(vertex_pass_trough_shader_file);
      std::string clipping_shader_source = Resources::lookup_shader(clipping_shader_file);
      std::string error_shader_source = Resources::lookup_shader(error_shader_file);
  #endif

  scm::gl::shader_ptr vertex_pass_through_stage(
    ctx.render_device->create_shader(scm::gl::STAGE_VERTEX_SHADER, vertex_pass_through_shader_source)
  );

  scm::gl::shader_ptr fragment_clipping_stage(
    ctx.render_device->create_shader(scm::gl::STAGE_FRAGMENT_SHADER, clipping_shader_source)
  );

  scm::gl::program_ptr clipping_shader_program(
    ctx.render_device->create_program({
      vertex_pass_through_stage,
      fragment_clipping_stage
    })
  );


  scm::gl::shader_ptr error_compute_stage(
    ctx.render_device->create_shader(scm::gl::STAGE_COMPUTE_SHADER, error_shader_source)
  );

  scm::gl::program_ptr compute_shader_program(
    ctx.render_device->create_program({
      error_compute_stage
    })
  );

  scm::gl::texture_image_ptr output_buffer(
    ctx.render_device->create_texture_2d(
      ctx.render_window->config.get_resolution(),
      scm::gl::FORMAT_R_32F, 1
    )
  );

  auto depth_stencil_state(ctx.render_device->create_depth_stencil_state(depth_stencil_state_));

  unsigned int invocations_x(ctx.render_window->config.get_resolution().x);
  unsigned int invocations_y(ctx.render_window->config.get_resolution().y);

  pass.process_ = [clipping_shader_program, compute_shader_program, output_buffer,
                   depth_stencil_state, invocations_x, invocations_y](
      PipelinePass &, PipelinePassDescription const& desc, Pipeline & pipe) {

    auto gbuffer(dynamic_cast<GBuffer*>(pipe.current_viewstate().target));

    if (gbuffer) {

      auto color_buffer(gbuffer->get_color_buffer());
      auto position_buffer(gbuffer->get_position_buffer());
      auto depth_buffer(gbuffer->get_depth_buffer());

      if (color_buffer && depth_buffer) {

        RenderContext const& ctx(pipe.get_context());

        scm::gl::context_all_guard guard(ctx.render_context);

        ////////////// CLIPPING STAGE ////////////////

        gbuffer->bind(ctx, false);
        gbuffer->set_viewport(ctx);

        ctx.render_context->set_depth_stencil_state(depth_stencil_state, 1);
        ctx.render_context->bind_program(clipping_shader_program);

        // private uniforms
        clipping_shader_program->uniform("color_buffer", color_buffer->get_handle(ctx));
        clipping_shader_program->uniform("position_buffer", position_buffer->get_handle(ctx));
        clipping_shader_program->uniform("depth_buffer", depth_buffer->get_handle(ctx));

        // user specified uniforms
        for (auto const& u : desc.uniforms) {
          u.second.apply(ctx, u.first, clipping_shader_program, 0);
        }

        ctx.render_context->apply();

        pipe.draw_quad();

        gbuffer->unbind(ctx);

        // flip read/write buffers to gain access to the currently written
        // buffer in the compute shader
        gbuffer->toggle_ping_pong();
        color_buffer = gbuffer->get_color_buffer();

        ////////////// COMPUTE STAGE ////////////////

        ctx.render_context->bind_program(compute_shader_program);

        // bind target buffer
        ctx.render_context->bind_image(output_buffer, scm::gl::FORMAT_R_32F,
                                       scm::gl::ACCESS_READ_WRITE,
                                       0);

        compute_shader_program->uniform_image("output_buffer", 0);

        // retrieve current photo
        auto camera_pos(math::get_translation(pipe.current_viewstate().camera.transform));
        auto closest_frustum(texstr::FrustumManagement::instance()->get_closest_frustum(
          scm::math::vec3d(camera_pos.x, camera_pos.y, camera_pos.z),
          scm::math::vec3d(1.0, 0.0, 1.0)
        ));

        std::shared_ptr<texstr::Texture> photo(nullptr);

        if (closest_frustum) {
          photo = texstr::TextureManagement::instance()->query_texture(
            ctx.render_device, ctx.render_context, closest_frustum
          );
        }

        // set uniforms
        compute_shader_program->uniform("color_buffer", color_buffer->get_handle(ctx));
        compute_shader_program->uniform("photo", photo->get_handle());


        ctx.render_context->apply();

        ctx.render_context->dispatch_compute(
          scm::math::vec3ui(invocations_x, invocations_y, 1u)
        );

        scm::float32 texture_data[invocations_x * invocations_y];

        ctx.render_context->retrieve_texture_data(output_buffer, 0, texture_data);

        // for (int i(0); i < invocations_x * invocations_y; ++i) {
        //   std::cout << texture_data[i] << std::endl;
        // }
        std::cout << texture_data[0] << std::endl;

        // restore gbuffer configuration
        gbuffer->toggle_ping_pong();

      }
    }





  };

  return pass;
}

}
