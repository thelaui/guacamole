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

#ifndef GUA_PLOD_DIRECT_RENDERER_HPP
#define GUA_PLOD_DIRECT_RENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

// guacamole headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ResourceFactory.hpp>

//external headers
#include <pbr/ren/cut_database_record.h>

namespace gua {

  class MaterialShader;
  class ShaderProgram;

  class PLODDirectRenderer {

  public:

    PLODDirectRenderer();

    void render(Pipeline& pipe, PipelinePassDescription const& desc);
    void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }
    void set_clamping_radius(float clamping_radius) { clamping_radius_ = clamping_radius; }

    void reload_programs();

 private:  //shader related auxiliary methods

  void          _load_shaders();
  void          _initialize_log_to_lin_conversion_pass_program();
  void          _initialize_direct_pass_program();
  void          _initialize_shadow_pass_program();

  void          _create_gpu_resources(gua::RenderContext const& ctx,
                                      scm::math::vec2ui const& render_target_dims,
                                      bool resize_resource_containers);

 private:  //out-of-core related auxiliary methods

  pbr::context_t _register_context_in_cut_update(gua::RenderContext const& ctx);

 private: //misc auxiliary methods
  bool _intersects(scm::gl::boxf const& bbox,
                   std::vector<math::vec4f> const& global_planes) const;

   std::vector<math::vec3> _get_frustum_corners_vs(gua::Frustum const& frustum) const;

 private:  //member variables

    //schism-GL states:
    //////////////////////////////////////////////////////////////////////////////////////
    scm::gl::rasterizer_state_ptr                no_backface_culling_rasterizer_state_;

    scm::gl::depth_stencil_state_ptr             depth_test_with_writing_depth_stencil_state_;

    //frustum dependent variables:
    /////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::map<pbr::model_t, std::vector<bool> > >  model_frustum_culling_results_;
    //misc:
    ////////////////////////////////////////////////////////////////////////////////////
    //unsigned int material_id_;  XXX still needed?
    scm::gl::quad_geometry_ptr                   fullscreen_quad_;

    bool                                         gpu_resources_already_created_;
    unsigned                                     previous_frame_count_;
    float                                        clamping_radius_;

    //context guard
    ////////////////////////////////////////////////////////////////////////////////////

    std::mutex                                   mutex_;
    bool                                         shaders_loaded_;

    //render target dependent resources
    unsigned                                                             current_rendertarget_width_;
    unsigned                                                             current_rendertarget_height_;

    //CPU resources
    std::vector<ShaderProgramStage>                                      direct_pass_shader_stages_;
    std::vector<ShaderProgramStage>                                      shadow_pass_shader_stages_;

    //additional GPU resources
    std::shared_ptr<ShaderProgram>                                       direct_pass_program_;

    std::shared_ptr<ShaderProgram>                                       shadow_pass_program_;

    SubstitutionMap                                                      global_substitution_map_;
    ResourceFactory                                                      factory_;
  };

}

#endif  // GUA_PLOD_DIRECT_RENDERER_HPP
