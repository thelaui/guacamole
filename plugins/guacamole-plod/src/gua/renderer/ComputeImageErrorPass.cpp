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
  rendermode_ = RenderMode::Custom;

}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> ComputeImageErrorPassDescription::make_copy() const {
  return std::make_shared<ComputeImageErrorPassDescription>(*this);
}

PipelinePass ComputeImageErrorPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{

  PipelinePass pass(*this, ctx, substitution_map);

  std::string shader_file("resources/shaders/projective_texturing/compute_image_error.glsl");

  #ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
      ResourceFactory factory;
      std::string shader_source = factory.read_shader_file(shader_file);
  #else
      std::string shader_source = Resources::lookup_shader(shader_file);
  #endif

  scm::gl::shader_ptr compute_stage(
    ctx.render_device->create_shader(scm::gl::STAGE_COMPUTE_SHADER, shader_source)
  );

  scm::gl::program_ptr compute_program(
    ctx.render_device->create_program({compute_stage})
  );

  scm::gl::texture_image_ptr output_buffer(
    ctx.render_device->create_texture_2d(
      ctx.render_window->config.get_resolution(),
      scm::gl::FORMAT_R_32F, 1
    )
  );


  unsigned int invocations_x(ctx.render_window->config.get_resolution().x);
  unsigned int invocations_y(ctx.render_window->config.get_resolution().y);

  pass.process_ = [compute_program, output_buffer,
                   invocations_x, invocations_y](
      PipelinePass &, PipelinePassDescription const&, Pipeline & pipe) {

    RenderContext const& ctx(pipe.get_context());

    scm::gl::context_all_guard(ctx.render_context);

    ctx.render_context->bind_program(compute_program);

    // bind output image
    ctx.render_context->bind_image(output_buffer, scm::gl::FORMAT_R_32F,
                                   scm::gl::ACCESS_READ_WRITE,
                                   0);

    compute_program->uniform_image("output_buffer", 0);


    auto gbuffer(dynamic_cast<GBuffer*>(pipe.current_viewstate().target));

    if (gbuffer) {

      // bind current rendered image
      auto rendered_image(gbuffer->get_color_buffer());

      if (rendered_image) {

        compute_program->uniform("rendered_image", rendered_image->get_handle(ctx));


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
      }
    }





  };

  return pass;
}

}
