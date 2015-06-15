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

#ifndef GUA_FRUSTUM_VISUALIZATION_PASS_HPP
#define GUA_FRUSTUM_VISUALIZATION_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua {

class Pipeline;

class GUA_DLL FrustumVisualizationPassDescription : public PipelinePassDescription {
 public:
  FrustumVisualizationPassDescription();
  std::shared_ptr<PipelinePassDescription> make_copy() const override;
  friend class Pipeline;

  void set_enabled(bool enabled);
  double get_query_radius() const { return query_radius_; }
  void set_query_radius(double query_radius);

 protected:
  PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;
  bool enabled_;
  double query_radius_;
};

}

#endif  // GUA_FRUSTUM_VISUALIZATION_PASS_HPP
