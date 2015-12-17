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

#ifndef GUA_COMPUTE_IMAGE_ERROR_PASS_HPP
#define GUA_COMPUTE_IMAGE_ERROR_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL ComputeImageErrorPassDescription : public PipelinePassDescription {
 public:
  ComputeImageErrorPassDescription();
  std::shared_ptr<PipelinePassDescription> make_copy() const override;
  friend class Pipeline;

  void set_clipping_parameters(gua::math::vec2f const& clipping_params);
  gua::math::vec2f const& get_clipping_parameters() const;

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

  gua::math::vec2f clipping_parameters_ = gua::math::vec2f(0.f, 0.f);
};

}

#endif  // GUA_COMPUTE_IMAGE_ERROR_PASS_HPP
