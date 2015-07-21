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
#include <sys/resource.h>

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

#include <pbr/ren/config.h>

#include <boost/filesystem.hpp>

#include <texture_stream/texture_stream.hpp>

#include "Navigator.hpp"

struct file_name_comp {
  bool operator() (std::string const& lhs, std::string const& rhs) const {
    auto lhs_num_pos(lhs.find("out_"));
    auto lhs_num_string(lhs.substr(lhs_num_pos + 4));
    lhs_num_string = lhs_num_string.substr(0, lhs_num_string.find("_"));
    auto lhs_num = gua::string_utils::from_string<int>(lhs_num_string);

    auto rhs_num_pos(rhs.find("out_"));
    auto rhs_num_string(rhs.substr(rhs_num_pos + 4));
    rhs_num_string = rhs_num_string.substr(0, rhs_num_string.find("_"));
    auto rhs_num = gua::string_utils::from_string<int>(rhs_num_string);

    return lhs_num < rhs_num;
  }
};

int main(int argc, char** argv) {
  /////////////////////////////////////////////////////////////////////////////
  // initialize guacamole
  /////////////////////////////////////////////////////////////////////////////

  gua::init(argc, argv);
  gua::Logger::enable_debug = false;
  texstr::Logger::state.verbose = false;

  /////////////////////////////////////////////////////////////////////////////
  // increase number of files that can be loaded in parallel
  /////////////////////////////////////////////////////////////////////////////

  const int max_load_count(280);
  int count(0);

  struct rlimit limit;

  limit.rlim_cur = PBR_CUT_UPDATE_NUM_LOADING_THREADS * max_load_count * 2;
  limit.rlim_max = limit.rlim_cur;
  std::cout << "rlimit " << limit.rlim_max << std::endl;
  if (setrlimit(RLIMIT_NOFILE, &limit) != 0) {
    printf("setrlimit() failed with errno=%d\n", errno);
    return 1;
  }

  /////////////////////////////////////////////////////////////////////////////
  // create scene
  /////////////////////////////////////////////////////////////////////////////

  // create scene graph object
  gua::SceneGraph graph("main_scenegraph");


  // create material for streets
  auto street_material_desc(std::make_shared<gua::MaterialShaderDescription>(
                                            "data/materials/StreetMaterial.gmd"));

  auto street_material_shader(std::make_shared<gua::MaterialShader>(
                                            "StreetMaterial",
                                            street_material_desc));

  gua::MaterialShaderDatabase::instance()->add(street_material_shader);

  auto street_material(street_material_shader->make_new_material());

  // create node to display filled in images
  gua::TriMeshLoader trimesh_loader;

  auto background_fill_transform = graph.add_node<gua::node::TransformNode>(
                                    "/", "background_fill_transform");

  auto background_fill = trimesh_loader.create_geometry_from_file(
                          "background_fill",
                          "data/objects/plane.obj",
                          street_material
                         );

  background_fill->rotate(90.0, 1.0, 0.0, 0.0);

  // graph.add_node("/background_fill_transform", background_fill);


  // configure plod-renderer and create point-based objects
  gua::PLODLoader plod_loader;

  plod_loader.set_upload_budget_in_mb(128);
  plod_loader.set_render_budget_in_mb(4048);
  plod_loader.set_out_of_core_budget_in_mb(4096);

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto model_offset = graph.add_node<gua::node::TransformNode>("/transform", "model_offset");

  auto setup_plod_node = [](std::shared_ptr<gua::node::PLODNode> const& node) {
    node->set_radius_scale(1.3f);
    node->set_enable_backface_culling_by_normal(false);
    // node->set_draw_bounding_box(true);
  };

  std::set<std::string, file_name_comp> model_files;
  std::vector<std::shared_ptr<gua::node::PLODNode>> plod_geometrys;

  // boost::filesystem::path model_path("/mnt/pitoti/lp/france/20121212/000/pointcloud/xyz/");
  boost::filesystem::path model_path("/media/laui/street_data/pointcloud/xyz/");
  // boost::filesystem::path model_path("/home/tosa2305/Desktop/thesis/data/untracked/point_clouds");

  if (is_directory(model_path)) {

    for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(model_path), {})) {

      auto model_filename = entry.path();

      if (model_filename.has_extension() && model_filename.extension() == ".kdn") {
        model_files.insert(model_filename.string());
      }
    }
  }

  for (auto file : model_files){
    if (count < max_load_count) {
      auto node = plod_loader.load_geometry(file + "_node",
                                           file,
                                           street_material,
                                           gua::PLODLoader::DEFAULTS |
                                           gua::PLODLoader::MAKE_PICKABLE);
      plod_geometrys.push_back(node);
      setup_plod_node(node);
      graph.add_node("/transform/model_offset", node);

      ++count;
    } else {
      break;
    }
  }

  model_offset->translate(-plod_geometrys[0]->get_bounding_box().center());
  auto offset_transform = model_offset->get_world_transform();

  /////////////////////////////////////////////////////////////////////////////
  // load frustum files
  /////////////////////////////////////////////////////////////////////////////

  std::set<std::string> frustum_files;
  std::vector<texstr::Frustum> frusta;

  // boost::filesystem::path frusta_path("/home/tosa2305/Desktop/thesis/data/untracked/frusta");
  // boost::filesystem::path frusta_path("/home/tosa2305/Desktop/thesis/data/untracked/frusta_subset_cam_0");
  boost::filesystem::path frusta_path("/home/laui/Dokumente/Studium/Master/data/untracked/frusta_subset_cam_0");
  // boost::filesystem::path frusta_path("/home/tosa2305/Desktop/thesis/data/untracked/frusta_subset_cam_0_new");

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
  int current_selection_mode(0);
  int background_fill_enabled(0);
  bool gui_visible(true);
  float current_blending_factor(1.f);
  int lens_enabled(0);
  gua::math::vec3 current_pick_pos(0.0);
  gua::math::vec3 current_pick_normal(0.0);
  float current_lens_radius(5.f);
  const scm::math::vec3d global_offset(-485784.23, -145.24, -5374211.66);

  /////////////////////////////////////////////////////////////////////////////
  // create scene camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  auto resolution = gua::math::vec2ui(1920, 1080);
  // auto resolution = gua::math::vec2ui(1280, 960);

  auto camera = graph.add_node<gua::node::CameraNode>("/", "cam");
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/cam/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_far_clip(5000.0f);
  camera->config.set_near_clip(0.001f);
  // camera->config.set_enable_frustum_culling(false);
  camera->config.mask().blacklist.add_tag("invisible");



  auto screen = graph.add_node<gua::node::ScreenNode>("/cam", "screen");
  // screen->data.set_size(gua::math::vec2(1.92f, 1.08f) * 0.01f);
  screen->data.set_size(gua::math::vec2(resolution.x, resolution.y) * 0.00001f);
  // screen->translate(0.0, 0.0, -0.5);
  // screen->data.set_size(gua::math::vec2(0.00824895, 0.006197296));
  screen->translate(0.0, 0.0, -0.0061637285428946);

  auto frustum_vis_pass(std::make_shared<gua::FrustumVisualizationPassDescription>());
  frustum_vis_pass->set_query_radius(50.0);
  frustum_vis_pass->set_tree_visualization_enabled(false);
  frustum_vis_pass->set_frustum_visualization_enabled(false);
  auto fill_pass = std::make_shared<gua::FullscreenPassDescription>();
  fill_pass->source_file("data/shaders/background_fill.frag");

  auto pipe = std::make_shared<gua::PipelineDescription>();

  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  // pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  //pipe->add_pass(std::make_shared<gua::BBoxPassDescription>());
  pipe->add_pass(std::make_shared<gua::PLODPassDescription>());
  pipe->add_pass(frustum_vis_pass);
  pipe->add_pass(std::make_shared<gua::TextureProjectionUpdatePassDescription>());
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(fill_pass);
  pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());

  // pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::SKYMAP_TEXTURE);
  pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::COLOR);
  // pipe->get_resolve_pass()->background_mode(gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  // pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/water_painted_noon.jpg");
  // pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/bath.jpg");
  // pipe->get_resolve_pass()->background_texture("/opt/guacamole/resources/skymaps/field.jpg");
  // pipe->get_resolve_pass()->background_texture("fill_texture");
  // pipe->get_resolve_pass()->background_color(gua::utils::Color3f(0.5f, 0.6f, 0.1f));
  pipe->get_resolve_pass()->background_color(gua::utils::Color3f(0.8f, 0.8f, 1.f));

  camera->set_pipeline_description(pipe);

  /////////////////////////////////////////////////////////////////////////////
  // create navigation
  /////////////////////////////////////////////////////////////////////////////

  Navigator navigator;
  bool navigator_active(false);
  bool gui_options_active(false);
  bool gui_map_active(false);
  navigator.set_transform(scm::math::mat4f(frusta[0].get_camera_transform()));

  /////////////////////////////////////////////////////////////////////////////
  // setup gui and map
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
    gui->add_javascript_getter("get_blending_factor", [&](){ return std::to_string(current_blending_factor);});
    gui->add_javascript_getter("get_blending_range", [&](){ return std::to_string(current_blending_range);});
    gui->add_javascript_getter("get_lens_radius", [&](){ return std::to_string(current_lens_radius);});

    gui->add_javascript_callback("set_selection_mode_camera");
    gui->add_javascript_callback("set_selection_mode_fragment");
    gui->add_javascript_callback("set_blending_mode_average");
    gui->add_javascript_callback("set_blending_mode_median");
    gui->add_javascript_callback("set_lens_enable");
    gui->add_javascript_callback("set_background_fill_enable");
    gui->add_javascript_callback("set_tree_vis_enable");
    gui->add_javascript_callback("set_frustum_vis_enable");
    gui->add_javascript_callback("set_query_radius");
    gui->add_javascript_callback("set_blending_factor");
    gui->add_javascript_callback("set_blending_range");
    gui->add_javascript_callback("set_lens_radius");

    gui->call_javascript("init");
  });

  gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
    if (callback == "set_selection_mode_camera"
     || callback == "set_selection_mode_fragment"
     || callback == "set_blending_mode_average"
     || callback == "set_blending_mode_median"
     || callback == "set_lens_enable"
     || callback == "set_background_fill_enable"
     || callback == "set_tree_vis_enable"
     || callback == "set_frustum_vis_enable") {
      std::stringstream str(params[0]);
      bool checked;
      str >> checked;

      if (callback == "set_selection_mode_camera") current_selection_mode = 0;
      if (callback == "set_selection_mode_fragment") current_selection_mode = 1;
      if (callback == "set_blending_mode_average") current_blending_mode = 0;
      if (callback == "set_blending_mode_median") current_blending_mode = 1;
      if (callback == "set_lens_enable") lens_enabled = checked ? 1 : 0;
      if (callback == "set_background_fill_enable") background_fill_enabled = checked ? 1 : 0;
      if (callback == "set_tree_vis_enable") frustum_vis_pass->set_tree_visualization_enabled(checked);
      if (callback == "set_frustum_vis_enable") frustum_vis_pass->set_frustum_visualization_enabled(checked);
    } else if (callback == "set_query_radius") {
      std::stringstream str(params[0]);
      double query_radius;
      str >> query_radius;
      frustum_vis_pass->set_query_radius(query_radius);
    } else if (callback == "set_blending_factor") {
      std::stringstream str(params[0]);
      str >> current_blending_factor;
    } else if (callback == "set_blending_range") {
      std::stringstream str(params[0]);
      str >> current_blending_range;
    } else if (callback == "set_lens_radius") {
      std::stringstream str(params[0]);
      str >> current_lens_radius;
    }
  });

  auto map = std::make_shared<gua::GuiResource>();
  map->init("map", "asset://gua/data/gui/map.html", gua::math::vec2(430, 760));

  auto map_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("map_quad");
  map_quad->data.texture() = "map";
  map_quad->data.size() = gua::math::vec2ui(430, 760);
  map_quad->data.anchor() = gua::math::vec2(-1.f, 0.f);

  graph.add_node("/", map_quad);

  map->on_loaded.connect([&]() {
    map->add_javascript_callback("set_camera_pos_utm");
    map->call_javascript("init");
  });

  map->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
    if (callback == "set_camera_pos_utm") {
      double easting, northing;
      std::stringstream easting_str(params[0]);
      easting_str >> easting;

      std::stringstream northing_str(params[1]);
      northing_str >> northing;

      scm::math::vec4d new_pos(-(easting + global_offset.x),
                                 global_offset.y,
                                 northing + global_offset.z,
                                 1.0);
      new_pos = offset_transform * new_pos;

      auto closest_frustum(texstr::FrustumManagement::instance()->get_closest_frustum(
        scm::math::vec3d(new_pos.x, new_pos.y, new_pos.z),
        scm::math::vec3d(1.0, 0.0, 1.0)
      ));

      if (closest_frustum) {
        navigator.set_transform(scm::math::mat4f(closest_frustum->get_camera_transform()));

        for (int i(0); i < frusta.size(); ++i) {
          if (frusta[i].get_image_file_name() == closest_frustum->get_image_file_name()) {
            current_frustum = i;
            break;
          }
        }
        navigator_active = false;
      }
    }
  });


  /////////////////////////////////////////////////////////////////////////////
  // create window and callback setup
  /////////////////////////////////////////////////////////////////////////////


  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("main_window", window);
  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    resolution = new_size;
    window->config.set_resolution(resolution);
    camera->config.set_resolution(resolution);
    screen->data.set_size(gua::math::vec2(0.00001f * resolution.x,
                                          0.00001f * resolution.y));
  });

  window->on_move_cursor.connect([&](gua::math::vec2 const& pos) {
    gua::math::vec2 hit_pos;
    gui_options_active = false;
    gui_map_active = false;
    if (gui_visible) {
      if (gui_quad->pixel_to_texcoords(pos, resolution, hit_pos)) {
        gui->inject_mouse_position_relative(hit_pos);
        gui_options_active = true;
      } else if (map_quad->pixel_to_texcoords(pos, resolution, hit_pos)) {
        map->inject_mouse_position_relative(hit_pos);
        gui_map_active = true;
      }
    }

    if (!gui_options_active && !gui_options_active) {
      navigator.set_mouse_position(gua::math::vec2i(pos));

      if (lens_enabled) {
        auto screen_space_pos = pos/gua::math::vec2(resolution.x, resolution.y) - 0.5;

        auto origin = screen->get_scaled_world_transform() *
                      gua::math::vec4(screen_space_pos.x, screen_space_pos.y, 0, 1);

        auto direction = scm::math::normalize(
                          origin - camera->get_cached_world_transform() *
                          gua::math::vec4(0,0,0,1)
                         ) * 100.0;

        auto up = screen->get_scaled_world_transform() *
                  gua::math::vec4(0.0, 1.0, 0.0, 0.0);

        auto picks = graph.ray_test(gua::Ray(origin, direction, 1.0),
                                    gua::PickResult::PICK_ONLY_FIRST_OBJECT |
                                    gua::PickResult::PICK_ONLY_FIRST_FACE |
                                    gua::PickResult::GET_WORLD_POSITIONS |
                                    gua::PickResult::GET_POSITIONS |
                                    gua::PickResult::GET_WORLD_NORMALS);

        if (!picks.empty()) {
          current_pick_pos = picks.begin()->world_position;
          current_pick_normal = picks.begin()->world_normal;
        }
      }
    }
  });

  window->on_button_press.connect([&](int button, int action, int mods){
    if (!gui_options_active && !gui_map_active) {
      navigator_active = true;
      navigator.set_mouse_button(button, action);
    } else if (gui_options_active) {
      gui->inject_mouse_button(gua::Button(button), action, mods);
    } else if (gui_map_active) {
      map->inject_mouse_button(gua::Button(button), action, mods);
    }
  });

  window->on_scroll.connect([&](gua::math::vec2 const& scroll){
    if (gui_map_active) {
      map->inject_mouse_wheel(scroll);
    }
  });

  window->on_char.connect([&](unsigned key){
    if (key == 'h') {
      if (gui_visible) {
        gui_quad->get_tags().add_tag("invisible");
        map_quad->get_tags().add_tag("invisible");
      } else {
        gui_quad->get_tags().remove_tag("invisible");
        map_quad->get_tags().remove_tag("invisible");
      }
      gui_visible = !gui_visible;
    }
  });

  window->on_key_press.connect([&](int key, int scancode, int action, int mods){

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
      // F5 to reload gui
      if (action == 1 && key == 294) {
        gui->reload();
        map->reload();
      }

      // F6 to reload pipeline
      if (action == 1 && key == 295) {
        fill_pass->touch();
      }

      bool position_changed(false);
      // arrow right
      if (key == 262) {
        current_frustum = std::min(current_frustum + 1, int(frusta.size() - 1));
        position_changed = true;
      // arrow left
      } else if (key == 263) {
        current_frustum = std::max(current_frustum - 1, 0);
        position_changed = true;
      }

      if (position_changed) {
        navigator.set_transform(scm::math::mat4f(frusta[current_frustum].get_camera_transform()));
        navigator_active = false;

        // update map marker
        auto pos(frusta[current_frustum].get_camera_position());
        pos = scm::math::inverse(offset_transform) * pos;
        pos = scm::math::vec3d(-(pos.x + global_offset.x),
                                 0.0,
                                 pos.z - global_offset.z
                                );
        map->call_javascript_arg_vector("set_position_marker_utm",
                                        {gua::string_utils::to_string(pos.x),
                                         gua::string_utils::to_string(pos.z)});

        background_fill_transform->set_world_transform(frusta[current_frustum].get_screen_transform());

        std::cout << frusta[current_frustum].get_image_file_name() << std::endl;
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
      // screen->set_world_transform(frusta[current_frustum].get_screen_transform());
      // screen->data.set_size(gua::math::vec2(1.0, 1.0));;
    }

    street_material->set_uniform("blending_range",  current_blending_range);
    street_material->set_uniform("blending_mode",   current_blending_mode);
    street_material->set_uniform("selection_mode",  current_selection_mode);
    street_material->set_uniform("blending_factor", current_blending_factor);
    street_material->set_uniform("pick_pos_and_radius",
                                  gua::math::vec4(current_pick_pos.x,
                                                  current_pick_pos.y,
                                                  current_pick_pos.z,
                                                  current_lens_radius));
    street_material->set_uniform("pick_normal", scm::math::normalize(current_pick_normal));
    street_material->set_uniform("lens_enabled", lens_enabled);
    int enable_background(0);
    if (background_fill_enabled == 1 && !navigator_active) enable_background = 1;
    fill_pass->uniform("enabled", enable_background);

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
