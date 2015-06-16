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

#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/PLODLoader.hpp>
#include <gua/node/PLODNode.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/PLODPass.hpp>
#include <gua/renderer/TriMeshPass.hpp>
#include <gua/renderer/SSAAPass.hpp>

#include <gua/renderer/BBoxPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/renderer/FrustumVisualizationPass.hpp>
#include <gua/renderer/TextureProjectionUpdatePass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>

#include <gua/gui.hpp>

#include <boost/filesystem.hpp>

#include <texture_stream/texture_stream.hpp>

#include "Navigator.hpp"

int main(int argc, char** argv) {
  /////////////////////////////////////////////////////////////////////////////
  // initialize guacamole
  /////////////////////////////////////////////////////////////////////////////

  gua::init(argc, argv);

  gua::Logger::enable_debug = false;

  /////////////////////////////////////////////////////////////////////////////
  // create scene
  /////////////////////////////////////////////////////////////////////////////

  // create scene graph object
  gua::SceneGraph graph("main_scenegraph");

  // configure plod-renderer and create point-based objects
  gua::PLODLoader plodLoader;
  gua::TriMeshLoader trimesh_loader;

  plodLoader.set_upload_budget_in_mb(32);
  plodLoader.set_render_budget_in_mb(2048);
  plodLoader.set_out_of_core_budget_in_mb(4096);

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto model_offset = graph.add_node<gua::node::TransformNode>("/transform", "model_offset");

  auto setup_plod_node = [](std::shared_ptr<gua::node::PLODNode> const& node) {
    node->set_radius_scale(0.8f);
    node->set_enable_backface_culling_by_normal(false);
    // node->set_draw_bounding_box(true);
  };

  auto projective_texturing_material_desc(std::make_shared<gua::MaterialShaderDescription>(
                                            "data/materials/ProjectiveTextureMaterial.gmd"));

  auto projective_texturing_material_shader(std::make_shared<gua::MaterialShader>(
                                            "ProjectiveTextureMaterial",
                                            projective_texturing_material_desc));

  gua::MaterialShaderDatabase::instance()->add(projective_texturing_material_shader);

  auto projective_texturing_material(projective_texturing_material_shader->make_new_material());

  std::set<std::string> model_files;
  std::vector<std::shared_ptr<gua::node::PLODNode>> plod_geometrys;

  boost::filesystem::path model_path("/mnt/pitoti/lp/france/20121212/000/pointcloud/xyz_new/");

  if (is_directory(model_path)) {

    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(model_path), {})) {
      auto model_filename = entry.path();

      if (model_filename.has_extension() && model_filename.extension() == ".kdn") {
        model_files.insert(model_filename.string());
      }
    }
  }

  for (auto file : model_files){
    auto node = plodLoader.load_geometry(file + "_node", file, projective_texturing_material, gua::PLODLoader::DEFAULTS);
    plod_geometrys.push_back(node);
    setup_plod_node(node);
    graph.add_node("/transform/model_offset", node);
  }

  model_offset->translate(-plod_geometrys[0]->get_bounding_box().center());
  auto offset_transform = model_offset->get_world_transform();

  /////////////////////////////////////////////////////////////////////////////
  // load frustum files
  /////////////////////////////////////////////////////////////////////////////

  std::set<std::string> frustum_files;
  std::vector<texstr::Frustum> frusta;

  // boost::filesystem::path frusta_path("/home/tosa2305/Desktop/thesis/data/untracked/frusta");
  boost::filesystem::path frusta_path("/home/tosa2305/Desktop/thesis/data/untracked/frusta_subset_cam_0");

  if (is_directory(frusta_path)) {

    for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(frusta_path), {})) {
      auto frustum_file_name = entry.path();

      if (frustum_file_name.has_extension() && frustum_file_name.extension() == ".frustum") {
        frustum_files.insert(frustum_file_name.string());
      }
    }
  }

  for (auto file : frustum_files){

    auto frustum = texstr::FrustumFactory::from_frustum_file(file);
    auto new_cam_trans = offset_transform * frustum.get_camera_transform();
    auto orig_screen_trans = scm::math::inverse(frustum.get_camera_transform()) * frustum.get_screen_transform();
    auto new_frustum = texstr::Frustum::perspective(
      new_cam_trans,
      new_cam_trans * orig_screen_trans,
      frustum.get_clip_near(),
      frustum.get_clip_far()
    );

    new_frustum.set_image_file_name(frustum.get_image_file_name());
    new_frustum.set_capture_time(frustum.get_capture_time());

    frusta.push_back(new_frustum);
  }

  texstr::FrustumManagement::instance()->register_frusta(frusta);

  int current_frustum(0);
  int current_blending_range(0);
  int current_blending_mode(0);

  /////////////////////////////////////////////////////////////////////////////
  // create scene camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  // auto resolution = gua::math::vec2ui(1920, 1080);
  auto resolution = gua::math::vec2ui(1280, 960);

  auto camera = graph.add_node<gua::node::CameraNode>("/", "cam");
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/cam/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_far_clip(100000.0f);
  // camera->config.set_far_clip(0.0061637285428946);
  camera->config.set_near_clip(0.01f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/cam", "screen");
  // screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  // screen->translate(0.0, 0.0, -2.0);
  screen->data.set_size(gua::math::vec2(0.00824895, 0.006197296));
  screen->translate(0.0, 0.0, -0.0061637285428946);


  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  // pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  //pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  auto frustum_vis_pass(std::make_shared<gua::FrustumVisualizationPassDescription>());
  frustum_vis_pass->set_query_radius(50.0);
  pipe->add_pass(frustum_vis_pass);
  pipe->add_pass(std::make_shared<gua::TextureProjectionUpdatePassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
  //pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

  pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  // pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/water_painted_noon.jpg");
  // pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/bath.jpg");
  pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/field.jpg");

  camera->set_pipeline_description(pipe);


  /////////////////////////////////////////////////////////////////////////////
  // setup gui
  /////////////////////////////////////////////////////////////////////////////

  auto gui = std::make_shared<gua::GuiResource>();
  gui->init("gui", "asset://gua/data/gui/gui.html", gua::math::vec2(330, 760));

  auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");
  gui_quad->data.texture() = "gui";
  gui_quad->data.size() = gua::math::vec2ui(330, 760);
  gui_quad->data.anchor() = gua::math::vec2(1.f, 0.f);

  graph.add_node("/", gui_quad);

  gui->on_loaded.connect([&]() {
    gui->add_javascript_getter("get_query_radius", [&](){ return std::to_string(frustum_vis_pass->get_query_radius());});

    gui->add_javascript_callback("set_blending_mode_average");
    gui->add_javascript_callback("set_blending_mode_median");
    gui->add_javascript_callback("set_frustum_vis_pass_enable");
    gui->add_javascript_callback("set_query_radius");

    gui->call_javascript("init");
  });

  gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
    if (callback == "set_blending_mode_average"
     || callback == "set_blending_mode_median"
     || callback == "set_frustum_vis_pass_enable") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;

      if (callback == "set_blending_mode_average") current_blending_mode = 0;
      if (callback == "set_blending_mode_median") current_blending_mode = 1;
      if (callback == "set_frustum_vis_pass_enable") frustum_vis_pass->set_enabled(checked);
    } if (callback == "set_query_radius") {
        std::stringstream str(params[0]);
        double query_radius;
        str >> query_radius;
        frustum_vis_pass->set_query_radius(query_radius);
      }
  });

  /////////////////////////////////////////////////////////////////////////////
  // create navigation
  /////////////////////////////////////////////////////////////////////////////

  Navigator navigator;
  bool navigator_active(false);
  navigator.set_transform(scm::math::mat4f(frusta[0].get_camera_transform()));

  /////////////////////////////////////////////////////////////////////////////
  // create window and callback setup
  /////////////////////////////////////////////////////////////////////////////


  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(gua::math::vec2(0.001f * new_size.x, 0.001f * new_size.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    gua::math::vec2 hit_pos;
    if (gui_quad->pixel_to_texcoords(pos, resolution, hit_pos)) {
      gui->inject_mouse_position_relative(hit_pos);
    } else {
      navigator.set_mouse_position(gua::math::vec2i(pos));
    }
  });

  window->on_button_press.connect([&](int button, int action, int mods){
    navigator_active = true;
    navigator.set_mouse_button(button, action);
    gui->inject_mouse_button(gua::Button(button), action, mods);
  });

  window->on_scroll.connect([&current_blending_range](gua::math::vec2 const& scroll){
    if (scroll.y > 0.0) {
      current_blending_range = std::min(current_blending_range + 1, 5);
    } else if (scroll.y < 0.0) {
      current_blending_range = std::max(current_blending_range - 1, 0);
    }
  });

  window->on_char.connect([&navigator, &navigator_active, &current_frustum, &frusta](unsigned key){
    if (key == 'e') {
      current_frustum = std::min(current_frustum + 1, int(frusta.size()));
      navigator_active = true;
    } else if (key == 'q') {
      current_frustum = std::max(current_frustum - 1, 0);
      navigator_active = true;
    }
  });

  window->on_key_press.connect([&navigator, &current_frustum, &frusta, &navigator_active](int key, int scancode, int action, int mods){

    auto gua_key(static_cast<gua::Key>(key));
    if (gua_key == gua::Key::W ||
        gua_key == gua::Key::S ||
        gua_key == gua::Key::A ||
        gua_key == gua::Key::D) {
      navigator_active = true;
      navigator.set_key_press(gua_key, action);
    }

    // check if key pressed
    if (action == 1 || action == 2) {
      // arrow right
      if (key == 262) {
        current_frustum = std::min(current_frustum + 1, int(frusta.size()));
        navigator.set_transform(scm::math::mat4f(frusta[current_frustum].get_camera_transform()));
        navigator_active = false;
      // arrow left
      } else if (key == 263) {
        current_frustum = std::max(current_frustum - 1, 0);
        navigator.set_transform(scm::math::mat4f(frusta[current_frustum].get_camera_transform()));
        navigator_active = false;
      }
    }
  });

  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);

  ticker.on_tick.connect([&]() {
    navigator.update();
    gua::Interface::instance()->update();

    if (navigator_active) {
      camera->set_transform(gua::math::mat4(navigator.get_transform()));
      // screen->set_transform(scm::math::make_translation(0.0, 0.0, -0.0061637285428946));
      // screen->data.set_size(gua::math::vec2(0.00824895, 0.006197296));
    } else {
      camera->set_transform(gua::math::mat4(frusta[current_frustum].get_camera_transform()));
      screen->set_world_transform(frusta[current_frustum].get_screen_transform());
      screen->data.set_size(gua::math::vec2(1.0, 1.0));;
    }

    projective_texturing_material->set_uniform("current_frustum", current_frustum);
    projective_texturing_material->set_uniform("blending_range",  current_blending_range);
    projective_texturing_material->set_uniform("blending_mode",   current_blending_mode);

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else {
      renderer.queue_draw({&graph});
    }
  });

  loop.start();

  return 0;
}
