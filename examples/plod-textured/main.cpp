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
    node->set_radius_scale(1.0f);
    node->set_enable_backface_culling_by_normal(false);
    // node->set_draw_bounding_box(true);
  };

  std::vector<std::shared_ptr<gua::node::PLODNode>> plod_geometrys;

  std::string const model_path("/mnt/pitoti/lp/france/20121212/000/pointcloud/xyz_new/");
  plod_geometrys.push_back(plodLoader.load_geometry(model_path + "out_0.kdn", gua::PLODLoader::DEFAULTS));


  for (auto p : plod_geometrys){
    setup_plod_node(p);
    graph.add_node("/transform/model_offset", p);
  }

  std::cout << plod_geometrys[0]->get_bounding_box().center() << std::endl;
  model_offset->translate(-plod_geometrys[0]->get_bounding_box().center());

  /////////////////////////////////////////////////////////////////////////////
  // create lighting
  /////////////////////////////////////////////////////////////////////////////
  // auto light = graph.add_node<gua::node::LightNode>("/transform/model_offset", "light");
  // light->data.set_type(gua::node::LightNode::Type::POINT);
  // light->data.set_enable_shadows(true);
  // light->data.set_brightness(30.f);
  // light->scale(3.f);
  // light->translate(0.5, 0.3, 1.3);

  /////////////////////////////////////////////////////////////////////////////
  // create scene camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  auto resolution = gua::math::vec2ui(1920, 1080);

  auto camera = graph.add_node<gua::node::CameraNode>("/", "cam");
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/cam/screen");
  camera->config.set_eye_dist(0.06f);
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_enable_stereo(false);
  camera->config.set_far_clip(100.0f);
  camera->config.set_near_clip(0.01f);
 //camera->set_pre_render_cameras({portal_camera});

  auto screen = graph.add_node<gua::node::ScreenNode>("/cam", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0.0, 0.0, -2.0);

  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  // pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  //pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
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

  auto start_transform(model_offset->get_world_transform());
  std::cout << start_transform << std::endl;
  // navigator.set_transform(scm::math::mat4f(start_transform));

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

  window->on_button_press.connect([&navigator](int button, int action, int mods){
    navigator.set_mouse_button(button, action);
  });

  window->on_char.connect([&navigator](unsigned key){
    navigator.set_key_press(key);
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

    camera->set_transform(gua::math::mat4(navigator.get_transform()));

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
