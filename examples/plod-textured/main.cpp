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
#include <gua/renderer/TexturedQuadPass.hpp>
#include <gua/renderer/TexturedScreenSpaceQuadPass.hpp>
#include <gua/renderer/ScreenSpacePickPass.hpp>

#include <gua/gui.hpp>

#include <pbr/ren/config.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <texture_stream/texture_stream.hpp>

#include "opencv2/opencv.hpp"

#include "Navigator.hpp"
#include "BruteForceOptimizer.hpp"
#include "SteepestDescentOptimizer.hpp"
#include "NewtonsMethodOptimizer.hpp"
#include "error_functions.hpp"

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
  // process arguments
  ////////////////////////////////////////////////////////////////////////////

  namespace po = boost::program_options;
  namespace fs = boost::filesystem;

  const std::string exec_name = (argc > 0) ? fs::basename(argv[0]) : "";
  po::options_description desc("Usage: " + exec_name + " [OPTION]...\n\n"
                               "Allowed Options");

  std::vector<std::string> frusta_paths_string({"/home/tosa2305/Desktop/thesis/data/untracked/france/frusta_subset_cam_0"});
  std::vector<std::string> model_paths_string({"/mnt/pitoti/lp/france/20121212/000/pointcloud/xyz/"});
  std::string optimization_output_path(".");
  int width(1920);
  int height(1080);
  double focal_length(0.005);
  double screen_width(width * 0.00001);
  double screen_height(height * 0.00001);
  bool optimization_enabled(false);
  std::string centroid_file_name("");

  desc.add_options()
    ("help", "print help message")
    ("frusta,f", po::value<std::vector<std::string>>(&frusta_paths_string)->multitoken()->default_value(frusta_paths_string), "specify a path to frustum files")
    ("models,m", po::value<std::vector<std::string>>(&model_paths_string)->multitoken()->default_value(model_paths_string), "specify paths to kdn trees")
    ("output,o", po::value<std::string>(&optimization_output_path)->default_value(optimization_output_path), "specify path to where optimized frusta shall be stored")
    ("width,w", po::value<int>(&width)->default_value(width), "specify width of the window's resolution")
    ("height,h", po::value<int>(&height)->default_value(height), "specify height of the window's resolution")
    ("focal-length,l", po::value<double>(&focal_length)->default_value(focal_length), "specify the focal length of the used camera in m")
    ("screen-width", po::value<double>(&screen_width)->default_value(screen_width), "specify the width of the virtual screen in m, i.e. the used camera's sensor width")
    ("screen-height", po::value<double>(&screen_height)->default_value(screen_height), "specify the height of the virtual screen in m, i.e. the used camera's sensor height")
    ("centroid", po::value<std::string>(&centroid_file_name)->default_value(centroid_file_name), "specify the path to an centroid file name")
    ("optimize", "enable optimization mode")
    ;

  po::variables_map vm;

  try {
    auto parsed_options = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    po::store(parsed_options, vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc;
      return 0;
    }

    optimization_enabled = vm.count("optimize");
  } catch (std::exception& e) {}

  /////////////////////////////////////////////////////////////////////////////
  // initialize guacamole
  /////////////////////////////////////////////////////////////////////////////

  gua::init(0, 0);
  gua::Logger::enable_debug = false;
  texstr::Logger::state.verbose = false;

  /////////////////////////////////////////////////////////////////////////////
  // increase number of files that can be loaded in parallel
  /////////////////////////////////////////////////////////////////////////////

  // const int max_load_count(100);
  const int max_load_count(180);
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

  int current_frustum(0);
  int current_blending_range(0);
  int current_blending_mode(0);
  int current_selection_mode(0);
  int background_fill_enabled(0);
  bool gui_visible(!optimization_enabled);
  bool map_visible(!optimization_enabled);
  bool navigator_active(false);
  bool gui_options_active(false);
  bool gui_map_active(false);
  bool picking_enabled(false);
  float current_blending_factor(optimization_enabled ? 0.f : 1.f);
  bool lens_enabled(false);
  bool measurement_enabled(false);
  gua::math::vec2 current_mouse_pos(0.0);
  gua::math::vec3 current_pick_pos(0.0);
  gua::math::vec3 current_pick_normal(0.0);
  float current_lens_radius(5.f);
  float current_splat_radius(1.f);
  scm::math::vec3d global_offset(0.0);
  bool screen_shot_taken(false);

  BruteForceOptimizer brute_force_optimizer;

  brute_force_optimizer.position_offset_range = 0.f;
  brute_force_optimizer.position_sampling_steps = 5;
  brute_force_optimizer.rotation_offset_range = 2.f;
  brute_force_optimizer.rotation_sampling_steps = 2;

  SteepestDescentOptimizer steepest_descent_optimizer;

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

  auto pick_proxy_transform = graph.add_node<gua::node::TransformNode>(
                                    "/", "pick_proxy_transform");

  // auto pick_proxy = trimesh_loader.create_geometry_from_file(
  //                                   "pick_proxy",
  //                                   "data/objects/plane.obj",
  //                                   gua::TriMeshLoader::MAKE_PICKABLE
  //                                  );

  // pick_proxy->rotate(90.0, 1.0, 0.0, 0.0);
  // pick_proxy->scale(100.f);
  // pick_proxy->get_tags().add_tag("invisible");

  // graph.add_node("/pick_proxy_transform", pick_proxy);


  // configure plod-renderer and create point-based objects
  gua::PLODLoader plod_loader;

  plod_loader.set_upload_budget_in_mb(128);
  plod_loader.set_render_budget_in_mb(4048);
  plod_loader.set_out_of_core_budget_in_mb(4096);

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  auto model_offset = graph.add_node<gua::node::TransformNode>("/transform", "model_offset");
  // transform->get_tags().add_tag("no_pick");

  auto setup_plod_node = [current_splat_radius](std::shared_ptr<gua::node::PLODNode> const& node) {
    node->set_radius_scale(current_splat_radius);
    node->set_enable_backface_culling_by_normal(false);
    // node->set_draw_bounding_box(true);
  };

  std::multiset<std::string, file_name_comp> model_files;
  std::vector<std::shared_ptr<gua::node::PLODNode>> plod_geometrys;

  for (auto model_path_string : model_paths_string) {
    boost::filesystem::path model_path(model_path_string);

    if (is_directory(model_path)) {

      for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(model_path), {})) {

        auto model_filename = entry.path();

        if (model_filename.has_extension() && model_filename.extension() == ".kdn") {
          model_files.insert(model_filename.string());
        }
      }
    }
  }


  for (auto file : model_files){
    if (count < max_load_count) {
      // auto node = plod_loader.load_geometry(file + "_node",
      //                                      file,
      //                                      street_material,
      //                                      gua::PLODLoader::DEFAULTS |
      //                                      gua::PLODLoader::MAKE_PICKABLE);
      auto node = plod_loader.load_geometry(file,
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
  // load centroid file
  /////////////////////////////////////////////////////////////////////////////

  if (centroid_file_name != "") {
    texstr::DSVParser parser;
    auto centroid_data = parser.read_file(centroid_file_name, ';');
    if (centroid_data.size() == 2) {
      auto centroid_values = centroid_data[1];
      double x,y,z;
      std::stringstream(centroid_values[0]) >> x;
      std::stringstream(centroid_values[1]) >> y;
      std::stringstream(centroid_values[2]) >> z;
      global_offset = scm::math::vec3d(-x,-y,-z);
      std::cout << "Loaded centroid " << global_offset << std::endl;
    }

  }

  /////////////////////////////////////////////////////////////////////////////
  // load frustum files
  /////////////////////////////////////////////////////////////////////////////

  std::set<std::string> frustum_files;
  std::vector<texstr::Frustum> frusta;
  std::stringstream            route_point_stream; // collect all coords to set route on map

  for (auto frusta_path_string : frusta_paths_string) {
    boost::filesystem::path frusta_path(frusta_path_string);

    if (is_directory(frusta_path)) {

      std::cout << frusta_path_string << std::endl;
      for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(frusta_path), {})) {
        auto frustum_file_name = entry.path();
        if (frustum_file_name.has_extension() && frustum_file_name.extension() == ".frustum") {
          frustum_files.insert(frustum_file_name.string());
        }
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

    new_frustum.set_homography(frustum.get_homography());
    new_frustum.set_image_file_name(frustum.get_image_file_name());
    new_frustum.set_image_dimensions(frustum.get_image_dimensions());
    new_frustum.set_capture_time(frustum.get_capture_time());

    frusta.push_back(new_frustum);

    auto pos(frustum.get_camera_position());
    pos = scm::math::vec3d(-(pos.x + global_offset.x),
                             0.0,
                             pos.z - global_offset.z
                            );

    route_point_stream << gua::string_utils::to_string(pos.x) << ";"
                       << gua::string_utils::to_string(pos.z) << "|";
  }

  texstr::FrustumManagement::instance()->register_frusta(frusta);

  /////////////////////////////////////////////////////////////////////////////
  // create optimization output path if necessary
  /////////////////////////////////////////////////////////////////////////////

  if (boost::filesystem::create_directories(optimization_output_path)) {
    std::cout << " Created missing directories on path " << optimization_output_path << "." << std::endl;
  }

  /////////////////////////////////////////////////////////////////////////////
  // create scene camera and pipeline
  /////////////////////////////////////////////////////////////////////////////

  auto resolution = gua::math::vec2ui(width, height);

  auto camera = graph.add_node<gua::node::CameraNode>("/", "cam");
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/cam/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("main_window");
  camera->config.set_output_texture_name("main_buffer");
  camera->config.set_far_clip(5000.0f);
  // camera->config.set_far_clip(10.0f);
  camera->config.set_near_clip(0.001f);
  // camera->config.set_far_clip(5.0f);
  // camera->config.set_near_clip(1.f);
  // camera->config.set_enable_frustum_culling(false);
  camera->config.mask().blacklist.add_tag("invisible");



  auto screen = graph.add_node<gua::node::ScreenNode>("/cam", "screen");
  screen->data.set_size(gua::math::vec2(screen_width, screen_height));
  // screen->translate(0.0, 0.0, -0.5);
  // if (optimization_enabled) {
  //   screen->data.set_size(gua::math::vec2(0.00824895, 0.006197296));
  // } else {
  //   screen->data.set_size(gua::math::vec2(resolution.x, resolution.y) * 0.00001f);
  // }
  screen->translate(0.0, 0.0, -focal_length);

  auto frustum_vis_pass(std::make_shared<gua::FrustumVisualizationPassDescription>());
  frustum_vis_pass->set_query_radius(20.0);
  frustum_vis_pass->set_tree_visualization_enabled(false);
  frustum_vis_pass->set_frustum_visualization_enabled(false);

  auto texturing_pass = std::make_shared<gua::FullscreenPassDescription>();
  texturing_pass->source_file("data/shaders/screen_space_street_texturing.frag");

  auto screen_space_pick_pass(std::make_shared<gua::ScreenSpacePickPassDescription>());
  screen_space_pick_pass->set_window_name("main_window");

  auto plod_pass = std::make_shared<gua::PLODPassDescription>();
  plod_pass->set_radius_clamping_enabled(optimization_enabled);

  auto pipe = std::make_shared<gua::PipelineDescription>();

  // pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(frustum_vis_pass);
  // pipe->add_pass(screen_space_pick_pass);
  pipe->add_pass(std::make_shared<gua::TexturedQuadPassDescription>());
  pipe->add_pass(plod_pass);
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  pipe->add_pass(std::make_shared<gua::ResolvePassDescription>());
  pipe->add_pass(texturing_pass);
  pipe->add_pass(std::make_shared<gua::TexturedScreenSpaceQuadPassDescription>());
  // pipe->add_pass(std::make_shared<gua::DebugViewPassDescription>());

  camera->set_pipeline_description(pipe);

  /////////////////////////////////////////////////////////////////////////////
  // create navigation
  /////////////////////////////////////////////////////////////////////////////

  Navigator navigator;
  navigator.set_transform(scm::math::mat4f(frusta[0].get_camera_transform()));

  /////////////////////////////////////////////////////////////////////////////
  // setup gui and map
  /////////////////////////////////////////////////////////////////////////////

  auto gui = std::make_shared<gua::GuiResource>();
  gui->init("gui", "asset://gua/data/gui/gui.html", gua::math::vec2(350, resolution.y));

  auto gui_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("gui_quad");
  gui_quad->data.texture() = "gui";
  gui_quad->data.size() = gua::math::vec2ui(350, resolution.y);
  gui_quad->data.anchor() = gua::math::vec2(1.f, 0.f);

  graph.add_node("/", gui_quad);

  gui->on_loaded.connect([&]() {
    gui->add_javascript_getter("get_query_radius", [&](){ return gua::to_string(frustum_vis_pass->get_query_radius());});
    gui->add_javascript_getter("get_splat_radius", [&](){ return gua::to_string(current_splat_radius);});
    gui->add_javascript_getter("get_blending_factor", [&](){ return gua::to_string(current_blending_factor);});
    gui->add_javascript_getter("get_blending_range", [&](){ return gua::to_string(current_blending_range);});
    gui->add_javascript_getter("get_lens_radius", [&](){ return gua::to_string(current_lens_radius);});
    gui->add_javascript_getter("get_position_range", [&](){ return gua::to_string(brute_force_optimizer.position_offset_range);});
    gui->add_javascript_getter("get_position_samples", [&](){ return gua::to_string(brute_force_optimizer.position_sampling_steps);});
    gui->add_javascript_getter("get_rotation_range", [&](){ return gua::to_string(brute_force_optimizer.rotation_offset_range);});
    gui->add_javascript_getter("get_rotation_samples", [&](){ return gua::to_string(brute_force_optimizer.rotation_sampling_steps);});

    gui->add_javascript_callback("set_selection_mode_camera");
    gui->add_javascript_callback("set_selection_mode_fragment");
    gui->add_javascript_callback("set_blending_mode_average");
    gui->add_javascript_callback("set_blending_mode_median");
    gui->add_javascript_callback("set_lens_enable");
    gui->add_javascript_callback("set_measurement_enable");
    gui->add_javascript_callback("set_background_fill_enable");
    gui->add_javascript_callback("set_tree_vis_enable");
    gui->add_javascript_callback("set_frustum_vis_enable");
    gui->add_javascript_callback("set_query_radius");
    gui->add_javascript_callback("set_splat_radius");
    gui->add_javascript_callback("set_blending_factor");
    gui->add_javascript_callback("set_blending_range");
    gui->add_javascript_callback("set_lens_radius");
    gui->add_javascript_callback("set_position_range");
    gui->add_javascript_callback("set_position_samples");
    gui->add_javascript_callback("set_rotation_range");
    gui->add_javascript_callback("set_rotation_samples");

    gui->call_javascript("init");
  });

  gui->on_javascript_callback.connect([&](std::string const& callback, std::vector<std::string> const& params) {
    if (callback == "set_selection_mode_camera"
     || callback == "set_selection_mode_fragment"
     || callback == "set_blending_mode_average"
     || callback == "set_blending_mode_median"
     || callback == "set_lens_enable"
     || callback == "set_measurement_enable"
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
      if (callback == "set_lens_enable") lens_enabled = checked;;
      if (callback == "set_measurement_enable") measurement_enabled = checked;;
      if (callback == "set_background_fill_enable") background_fill_enabled = checked ? 1 : 0;
      if (callback == "set_tree_vis_enable") frustum_vis_pass->set_tree_visualization_enabled(checked);
      if (callback == "set_frustum_vis_enable") frustum_vis_pass->set_frustum_visualization_enabled(checked);
    } else if (callback == "set_query_radius") {
      std::stringstream str(params[0]);
      double query_radius;
      str >> query_radius;
      frustum_vis_pass->set_query_radius(query_radius);
    } else if (callback == "set_splat_radius") {
      std::stringstream str(params[0]);
      str >> current_splat_radius;
      for (auto& node : plod_geometrys) {
        node->set_radius_scale(current_splat_radius);
      }
    } else if (callback == "set_blending_factor") {
      std::stringstream str(params[0]);
      str >> current_blending_factor;
    } else if (callback == "set_blending_range") {
      std::stringstream str(params[0]);
      str >> current_blending_range;
    } else if (callback == "set_lens_radius") {
      std::stringstream str(params[0]);
      str >> current_lens_radius;
    } else if (callback == "set_position_range") {
      std::stringstream str(params[0]);
      str >> brute_force_optimizer.position_offset_range;
    } else if (callback == "set_position_samples") {
      std::stringstream str(params[0]);
      str >> brute_force_optimizer.position_sampling_steps;
    } else if (callback == "set_rotation_range") {
      std::stringstream str(params[0]);
      str >> brute_force_optimizer.rotation_offset_range;
    } else if (callback == "set_rotation_samples") {
      std::stringstream str(params[0]);
      str >> brute_force_optimizer.rotation_sampling_steps;
    }
  });

  auto map = std::make_shared<gua::GuiResource>();
  map->init("map", "asset://gua/data/gui/map.html", gua::math::vec2(430, resolution.y));

  auto map_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("map_quad");
  map_quad->data.texture() = "map";
  map_quad->data.size() = gua::math::vec2ui(430, resolution.y);
  map_quad->data.anchor() = gua::math::vec2(-1.f, 0.f);

  graph.add_node("/", map_quad);

  map->on_loaded.connect([&]() {
    map->add_javascript_callback("set_camera_pos_utm");
    map->call_javascript("init");
    map->call_javascript("set_route_points", route_point_stream.str());
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

        auto ground_transform(scm::math::make_translation(
          frusta[current_frustum].get_camera_position() -
          scm::math::vec3d(0.0, 2.83, 0.0)
        ));
        pick_proxy_transform->set_world_transform(ground_transform);
      }
    }
  });

  //measurement

  auto measurement_marker_1(std::make_shared<gua::node::TransformNode>("measurement_marker_1"));
  graph.add_node("/", measurement_marker_1);
  measurement_marker_1->translate(0.0, 1000.0, 0.0);
  auto measurement_marker_2(std::make_shared<gua::node::TransformNode>("measurement_marker_2"));
  graph.add_node("/", measurement_marker_2);
  measurement_marker_2->translate(0.0, 1000.0, 0.0);

  auto ruler = std::make_shared<gua::GuiResource>();
  ruler->init("ruler", "asset://gua/data/gui/ruler.html", gua::math::vec2(1920, 60));

  auto ruler_quad = std::make_shared<gua::node::TexturedQuadNode>("ruler_quad");
  ruler_quad->data.texture() = "ruler";
  ruler_quad->data.size() = gua::math::vec2(5.f, 1.f);
  ruler_quad->translate(0.0, 1000.0, 0.0);

  auto ruler_offset(graph.add_node<gua::node::TransformNode>("/", "ruler_offset"));
  graph.add_node("/ruler_offset", ruler_quad);

  auto measurement_text = std::make_shared<gua::GuiResource>();
  measurement_text->init("measurement_text", "asset://gua/data/gui/measurement_text.html", gua::math::vec2(100, 100));

  auto measurement_text_quad = std::make_shared<gua::node::TexturedScreenSpaceQuadNode>("measurement_text_quad");
  measurement_text_quad->data.texture() = "measurement_text";
  measurement_text_quad->data.size() = gua::math::vec2i(100, 100);
  measurement_text_quad->data.anchor() = gua::math::vec2(0.f, 0.f);
  measurement_text_quad->data.offset() = gua::math::vec2(0.f, 0.f);

  graph.add_node("/", measurement_text_quad);

  //gui visibility helper lambdas
  auto update_gui_visibility = [&](){
    if (!gui_visible) {
      gui_quad->get_tags().add_tag("invisible");
      measurement_text_quad->get_tags().add_tag("invisible");
      ruler_quad->get_tags().add_tag("invisible");
    } else {
      gui_quad->get_tags().remove_tag("invisible");
      measurement_text_quad->get_tags().remove_tag("invisible");
      ruler_quad->get_tags().remove_tag("invisible");
    }

    if (!map_visible) {
      map_quad->get_tags().add_tag("invisible");
    } else {
      map_quad->get_tags().remove_tag("invisible");
    }
  };

  auto update_map_marker = [&](){
    auto pos(frusta[current_frustum].get_camera_position());
    pos = scm::math::inverse(offset_transform) * pos;
    pos = scm::math::vec3d(-(pos.x + global_offset.x),
                             0.0,
                             pos.z - global_offset.z
                            );
    map->call_javascript_arg_vector("set_position_marker_utm",
                                    {gua::string_utils::to_string(pos.x),
                                     gua::string_utils::to_string(pos.z)});
  };

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
    current_mouse_pos = pos;

    if (gui_visible || map_visible) {
      if (gui_quad->pixel_to_texcoords(current_mouse_pos, resolution, hit_pos)) {
        gui->inject_mouse_position_relative(hit_pos);
        gui_options_active = true;
      } else if (map_quad->pixel_to_texcoords(current_mouse_pos, resolution, hit_pos)) {
        map->inject_mouse_position_relative(hit_pos);
        gui_map_active = true;
      }
    }

    if (!gui_options_active && !gui_options_active) {
      navigator.set_mouse_position(gua::math::vec2i(current_mouse_pos));
    }
  });

  window->on_button_press.connect([&](int button, int action, int mods){
    if (!gui_options_active && !gui_map_active) {
      if (measurement_enabled) {
        if (action == 1) {
          if (button == 0) {
            measurement_marker_1->translate(current_pick_pos - measurement_marker_1->get_world_position());
          } else if (button == 1) {
            measurement_marker_2->translate(current_pick_pos - measurement_marker_2->get_world_position());
          }
          auto marker_to_marker(measurement_marker_2->get_world_position() - measurement_marker_1->get_world_position());
          auto distance(scm::math::length(marker_to_marker));

          auto new_transform(scm::math::make_look_at_matrix_inv(
                              measurement_marker_1->get_world_position(),
                              measurement_marker_2->get_world_position(),
                              gua::math::vec3(0.0, 1.0, 0.0))
                             );

          ruler_offset->set_world_transform(new_transform);
          ruler_quad->data.size().x = distance;

          ruler_quad->set_transform(
            scm::math::make_translation(
              0.0,
              ruler_quad->data.size().y * 0.5,
              -ruler_quad->data.size().x * 0.5
            ) *
            scm::math::make_rotation(90.0, 0.0, 1.0, 0.0)
          );

          measurement_text->call_javascript("set_text", gua::to_string(distance) + "m");
        }
      } else {
        navigator_active = true;
        navigator.set_mouse_button(button, action);
      }
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
    if (key == 'g') {
      gui_visible = !gui_visible;

    } else if (key == 'm') {
      map_visible = !map_visible;
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
        ruler->reload();
        measurement_text->reload();
      }

      // F6 to reload pipeline
      if (action == 1 && key == 295) {
        texturing_pass->touch();
      }

      // F7 for screen shot
      if (action == 1 && key == 296) {
        // window->take_screen_shot();
        screen_shot_taken = true;
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

        update_map_marker();

        auto ground_transform(scm::math::make_translation(
          frusta[current_frustum].get_camera_position() -
          scm::math::vec3d(0.0, 2.83, 0.0)
        ));
        pick_proxy_transform->set_world_transform(ground_transform);

        std::cout << frusta[current_frustum].get_image_file_name() << std::endl;
      }
    }
  });

  window->open();

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0/500.0);
  int frame_count(0);

  ticker.on_tick.connect([&]() {
    ++frame_count;
    navigator.update();
    gua::Interface::instance()->update();
    update_gui_visibility();

    if (navigator_active) {
      camera->set_transform(gua::math::mat4(navigator.get_transform()));
      // screen->set_transform(scm::math::make_translation(0.0, 0.0, -0.0061637285428946));
      // screen->data.set_size(gua::math::vec2(0.00824895, 0.006197296));
      auto camera_pos(camera->get_world_position());
      int frustum_id(-1);
      auto closest_frustum(texstr::FrustumManagement::instance()->get_closest_frustum(
        scm::math::vec3d(camera_pos.x, camera_pos.y, camera_pos.z),
        scm::math::vec3d(1.0, 0.0, 1.0),
        frustum_id
      ));

      if (frustum_id != -1) {
        current_frustum = frustum_id;
        if (frame_count % 50 == 0) {
          update_map_marker();
        }
      }

    } else {
      camera->set_transform(
        gua::math::mat4(frusta[current_frustum].get_camera_transform()) *
        scm::math::make_rotation(-90.0, 1.0, 0.0, 0.0)
      );

    }

    if (screen_shot_taken) {
      screen_shot_taken = false;

      steepest_descent_optimizer.initial_transform = scm::math::mat4f(camera->get_transform());
      brute_force_optimizer.initial_transform = scm::math::mat4f(camera->get_transform());

      cv::Size blur_kernel(11,11);

      auto retrieve_photo = [&]() {
        cv::Mat photo(cv::imread(frusta[current_frustum].get_image_file_name(), CV_LOAD_IMAGE_GRAYSCALE));
        cv::resize(photo, photo, cv::Size(resolution.x, resolution.y));
        cv::GaussianBlur(photo, photo, blur_kernel, 0.0);
        return photo;
      };

      auto retrieve_screen_shot= [&](scm::math::mat4d const& new_transform) {
        camera->set_transform(new_transform);
        window->take_screen_shot();
        renderer.draw_single_threaded({&graph});

        std::vector<char> data;
        window->retrieve_screen_shot_data(data);
        cv::Mat screen_shot(resolution.y, resolution.x, CV_32FC3, data.data());
        cv::flip(screen_shot, screen_shot, 0); //flip around x axis
        cv::cvtColor(screen_shot, screen_shot, CV_BGR2GRAY); //convert from bgr to rgb color space
        screen_shot.convertTo(screen_shot, CV_8UC1, 255.0);
        cv::GaussianBlur(screen_shot, screen_shot, blur_kernel, 0.0);
        cv::equalizeHist(screen_shot, screen_shot);
        return screen_shot;
      };

      steepest_descent_optimizer.retrieve_photo = retrieve_photo;
      steepest_descent_optimizer.retrieve_screen_shot = retrieve_screen_shot;
      // steepest_descent_optimizer.error_function = summed_distances_to_closest_line;
      steepest_descent_optimizer.error_function = intensity_znssd;

      brute_force_optimizer.retrieve_photo = retrieve_photo;
      brute_force_optimizer.retrieve_screen_shot = retrieve_screen_shot;
      brute_force_optimizer.error_function = summed_distances_to_closest_line;
      // steepest_descent_optimizer.error_function = intensity_znssd;

      scm::math::mat4d optimal_transform(scm::math::mat4d::identity());
      scm::math::mat4d optimal_difference(scm::math::mat4d::identity());
      steepest_descent_optimizer.run(optimal_transform, optimal_difference);
      // brute_force_optimizer.run(optimal_transform, optimal_difference);

      for (int i(current_frustum); i < frusta.size(); ++i) {
        auto new_cam_trans = scm::math::mat4d::identity();
        // use optimized transform for current frustum and optimal difference for all subsequent others
        if (i != current_frustum) {
          auto new_transform = frusta[i].get_camera_transform() * scm::math::make_rotation(-90.0, 1.0, 0.0, 0.0) * scm::math::mat4d(optimal_difference);
          new_cam_trans = scm::math::mat4d(new_transform) * scm::math::make_rotation(90.0, 1.0, 0.0, 0.0);
        } else {
          new_cam_trans = scm::math::mat4d(optimal_transform) * scm::math::make_rotation(90.0, 1.0, 0.0, 0.0);
        }

        auto orig_screen_trans = scm::math::inverse(frusta[i].get_camera_transform()) * frusta[i].get_screen_transform();
        auto new_frustum = texstr::Frustum::perspective(
          new_cam_trans,
          new_cam_trans * orig_screen_trans,
          frusta[i].get_clip_near(),
          frusta[i].get_clip_far()
        );

        new_frustum.set_homography(frusta[i].get_homography());
        new_frustum.set_image_file_name(frusta[i].get_image_file_name());
        new_frustum.set_image_dimensions(frusta[i].get_image_dimensions());
        new_frustum.set_capture_time(frusta[i].get_capture_time());

        frusta[i] = new_frustum;

        // save to file

        boost::filesystem::path frustum_path(new_frustum.get_image_file_name());
        std::string out_file_name(optimization_output_path + "/" +
                                  frustum_path.filename().string() + ".frustum");

        new_cam_trans = scm::math::inverse(offset_transform) * new_cam_trans;
        new_frustum = texstr::Frustum::perspective(
          new_cam_trans,
          new_cam_trans * orig_screen_trans,
          frusta[i].get_clip_near(),
          frusta[i].get_clip_far()
        );

        new_frustum.set_homography(frusta[i].get_homography());
        new_frustum.set_image_file_name(frusta[i].get_image_file_name());
        new_frustum.set_image_dimensions(frusta[i].get_image_dimensions());
        new_frustum.set_capture_time(frusta[i].get_capture_time());

        std::fstream ofstr(out_file_name, std::ios::out);
        if (ofstr.good()) {
          ofstr << texstr::FrustumFactory::to_string(new_frustum) << std::endl;

        } else {
          std::cout << "Could not open output file " +  out_file_name + "!" << std::endl;
        }
        ofstr.close();
      }

      // register new frusta
      texstr::FrustumManagement::instance()->reset();
      texstr::FrustumManagement::instance()->register_frusta(frusta);
    }


    picking_enabled = lens_enabled || measurement_enabled;

    if (picking_enabled) {
      auto screen_space_pos = current_mouse_pos/gua::math::vec2(resolution.x, resolution.y) - 0.5;

      auto origin = screen->get_scaled_world_transform() *
                    gua::math::vec4(screen_space_pos.x, screen_space_pos.y, 0, 1);

      auto direction = scm::math::normalize(
                        origin - camera->get_cached_world_transform() *
                        gua::math::vec4(0,0,0,1)
                       ) * 100.0;

      auto picks = graph.ray_test(gua::Ray(origin, direction, 1.0),
                                  gua::PickResult::PICK_ONLY_FIRST_OBJECT |
                                  gua::PickResult::PICK_ONLY_FIRST_FACE |
                                  gua::PickResult::GET_WORLD_POSITIONS |
                                  gua::PickResult::GET_POSITIONS |
                                  gua::PickResult::GET_WORLD_NORMALS,
                                  gua::Mask({}, {"no_pick"}));

      if (!picks.empty()) {
        current_pick_pos = picks.begin()->world_position;
        current_pick_normal = picks.begin()->world_normal;
      }
    }

    auto marker_center((measurement_marker_2->get_world_position() + measurement_marker_1->get_world_position()) * 0.5);
    gua::math::vec4 text_center(marker_center.x,
                                marker_center.y + ruler_quad->data.size().y * 0.8,
                                marker_center.z,
                                1.0);

    auto rendering_frustum(camera->get_rendering_frustum(graph, gua::CameraMode::CENTER));
    auto text_center_screen_space((rendering_frustum.get_projection() * rendering_frustum.get_view()) * text_center);

    if (gui_visible) {
      if (text_center_screen_space.z < 0.0) {
        measurement_text_quad->get_tags().add_tag("invisible");
      } else {
        measurement_text_quad->get_tags().remove_tag("invisible");
        text_center_screen_space /= text_center_screen_space.w;
        // text_center_screen_space = text_center_screen_space * 0.5 + 0.5;
        measurement_text_quad->data.offset() = gua::math::vec2(
          text_center_screen_space.x * resolution.x,
          text_center_screen_space.y * resolution.y
        ) * 0.5;
      }
    }

    street_material->set_uniform("blending_range",  current_blending_range);
    street_material->set_uniform("blending_mode",   current_blending_mode);
    street_material->set_uniform("selection_mode",  current_selection_mode);
    street_material->set_uniform("pick_pos_and_radius",
                                  gua::math::vec4(current_pick_pos.x,
                                                  current_pick_pos.y,
                                                  current_pick_pos.z,
                                                  current_lens_radius));
    street_material->set_uniform("pick_normal", current_pick_normal);
    street_material->set_uniform("lens_enabled", lens_enabled ? 1 : 0);

    texturing_pass->uniform("selection_mode",  current_selection_mode);
    texturing_pass->uniform("blending_factor", current_blending_factor);
    texturing_pass->uniform("clipping_params", scm::math::vec2f(7.f, float(camera->get_world_position().y - 2.65f)));
    texturing_pass->uniform("clipping_enabled", optimization_enabled ? 1 : 0);

    window->process_events();
    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else {
      if (optimization_enabled) {
        renderer.draw_single_threaded({&graph});
      } else {
        renderer.queue_draw({&graph});
      }
    }
  });

  loop.start();

  return 0;
}
