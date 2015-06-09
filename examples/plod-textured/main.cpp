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
    auto node = plodLoader.load_geometry(file, gua::PLODLoader::DEFAULTS);
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

  boost::filesystem::path frusta_path("/home/tosa2305/Desktop/thesis/data/untracked/frusta");

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
    frusta.push_back(texstr::Frustum::perspective(
      new_cam_trans,
      new_cam_trans * orig_screen_trans,
      frustum.get_clip_near(),
      frustum.get_clip_far()
    ));
  }

  texstr::FrustumManagement::register_frusta(frusta);

  int current_frustum(0);

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
  pipe->add_pass(std::make_shared<gua::FrustumVisualizationPassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(std::make_shared<gua::SSAAPassDescription>());
  //pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

  pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/water_painted_noon.jpg");

  camera->set_pipeline_description(pipe);

  /////////////////////////////////////////////////////////////////////////////
  // create navigation
  /////////////////////////////////////////////////////////////////////////////

  Navigator navigator;
  bool navigator_active(true);
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

  window->on_move_cursor.connect([&navigator](gua::math::vec2 const& pos) {
    navigator.set_mouse_position(gua::math::vec2i(pos));
  });

  window->on_button_press.connect([&navigator, &navigator_active](int button, int action, int mods){
    navigator_active = true;
    navigator.set_mouse_button(button, action);
  });

  window->on_char.connect([&navigator, &navigator_active](unsigned key){
    if (key == 'w' || key == 'a' || key == 's' || key == 'd') {
      navigator_active = true;
    }
    navigator.set_key_press(key);
  });

  window->on_key_press.connect([&navigator, &current_frustum, &frusta, &navigator_active](int key, int scancode, int action, int mods){
    // check if key pressed
    if (action == 1) {
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

  std::size_t ctr = 0;

  auto last_frame_time = std::chrono::steady_clock::now();

  ticker.on_tick.connect([&]() {
    navigator.update();

    if (navigator_active) {
      camera->set_transform(gua::math::mat4(navigator.get_transform()));
    } else {
      camera->set_transform(gua::math::mat4(frusta[current_frustum].get_camera_transform()));
    }

    static unsigned framecounter = 0;
    ++framecounter;

    auto current_time = std::chrono::steady_clock::now();
    double milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time).count() / 1000.0;

    last_frame_time = current_time;


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
