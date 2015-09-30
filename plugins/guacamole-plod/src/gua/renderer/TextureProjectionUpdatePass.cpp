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

  // pass.process_ = [uniform_block_, projection_view_mats_, projection_textures_](PipelinePass&, PipelinePassDescription const&, Pipeline& pipe) {

  //   RenderContext const& ctx(pipe.get_context());



  // };

  return pass;
}

}
