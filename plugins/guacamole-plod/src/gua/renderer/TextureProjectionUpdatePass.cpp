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
#include <gua/renderer/TextureProjectionUpdatePass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/TextureProjectionUniformBlock.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/utils/Logger.hpp>

 #include <texture_stream/core/core.hpp>

namespace gua {

TextureProjectionUpdatePassDescription::TextureProjectionUpdatePassDescription() : PipelinePassDescription() {
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> TextureProjectionUpdatePassDescription::make_copy() const {
  return std::make_shared<TextureProjectionUpdatePassDescription>(*this);
}

PipelinePass TextureProjectionUpdatePassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};

  // auto uniform_block_(std::make_shared<TextureProjectionUniformBlock>(ctx.render_device));

  // auto frusta = texstr::FrustumManagement::fetch_frusta();
  // std::vector<math::mat4f> projection_view_mats_;
  // std::vector<math::vec2ui> projection_textures_;

  // for (int i(0); i < MAX_PROJECTIVE_TEXTURE_COUNT; ++i) {
  //   projection_view_mats_.push_back(math::mat4f(frusta[i].get_projection() * frusta[i].get_view()));

  //   math::vec2ui texture_handle(0);
  //   if (!TextureDatabase::instance()->contains(frusta[i].get_image_file_name())) {
  //     auto new_tex = std::make_shared<Texture2D>(frusta[i].get_image_file_name());
  //     TextureDatabase::instance()->add(frusta[i].get_image_file_name(), new_tex);
  //     texture_handle = new_tex->get_handle(ctx);
  //   } else {
  //     texture_handle = TextureDatabase::instance()->lookup(frusta[i].get_image_file_name())->get_handle(ctx);
  //   }
  //   projection_textures_.push_back(texture_handle);
  // }

  // pass.process_ = [uniform_block_, projection_view_mats_, projection_textures_](PipelinePass&, PipelinePassDescription const&, Pipeline& pipe) {

  //   RenderContext const& ctx(pipe.get_context());

  //   uniform_block_->update(ctx, projection_view_mats_, projection_textures_);
  //   ctx.render_context->bind_uniform_buffer(uniform_block_->block().block_buffer(), 2);

  // };

  return pass;
}

}
