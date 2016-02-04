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
#include <gua/renderer/PLODPass.hpp>

// guacamole headers
#include <gua/renderer/PLODResource.hpp>
#include <gua/renderer/PLODRenderer.hpp>
#include <gua/renderer/PLODDirectRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>

#include <scm/gl_core/shader_objects.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

PLODPassDescription::PLODPassDescription()
  : PipelinePassDescription()
{
  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = false;
  enable_for_shadows_ = true;
  rendermode_ = RenderMode::Custom;
  radius_clamping_enabled_ = false;
  render_method_ = RenderMethod::DIRECT;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> PLODPassDescription::make_copy() const {
  return std::make_shared<PLODPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////

void PLODPassDescription::set_radius_clamping_enabled(bool enabled) {
  radius_clamping_enabled_ = enabled;
  touch();
}

////////////////////////////////////////////////////////////////////////////////

void PLODPassDescription::set_render_method(RenderMethod render_method) {
  render_method_ = render_method;
  touch();
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass PLODPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{ *this, ctx, substitution_map };

  auto direct_renderer = std::make_shared<PLODDirectRenderer>();
  auto blended_renderer = std::make_shared<PLODRenderer>();
  blended_renderer->set_global_substitution_map(substitution_map);

  auto radius_clamping_enabled(radius_clamping_enabled_);
  auto render_method(render_method_);
  pass.process_ = [direct_renderer, blended_renderer, radius_clamping_enabled, render_method](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {
    if (render_method == RenderMethod::DIRECT) {
      direct_renderer->set_radius_clamping_enabled(radius_clamping_enabled);
      direct_renderer->render(pipe, desc);
    } else if (render_method == RenderMethod::BLENDED) {
      blended_renderer->set_radius_clamping_enabled(radius_clamping_enabled);
      blended_renderer->render(pipe, desc);
    }
  };

  return pass;
}

}
