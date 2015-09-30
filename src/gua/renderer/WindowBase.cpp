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
#include <gua/renderer/WindowBase.hpp>

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <sstream>
#include <iostream>

namespace gua {

std::string subroutine_from_mode(WindowBase::TextureDisplayMode mode) {
  switch (mode) {
    case WindowBase::RED:
      return "get_red";
      break;
    case WindowBase::GREEN:
      return "get_green";
      break;
    case WindowBase::CYAN:
      return "get_cyan";
      break;
    case WindowBase::CHECKER_EVEN:
      return "get_checker_even";
      break;
    case WindowBase::CHECKER_ODD:
      return "get_checker_odd";
      break;
    default:
      return "get_full";
  }
}

////////////////////////////////////////////////////////////////////////////////

std::atomic_uint WindowBase::last_context_id_{ 0 };
std::mutex WindowBase::last_context_id_mutex_{};

////////////////////////////////////////////////////////////////////////////////

WindowBase::WindowBase(Configuration const& configuration)
    : rendering_fps(1.0f),
      config(configuration),
      fullscreen_shader_(),
      fullscreen_quad_(),
      depth_stencil_state_(),
      warpRR_(nullptr),
      warpGR_(nullptr),
      warpBR_(nullptr),
      warpRL_(nullptr),
      warpGL_(nullptr),
      warpBL_(nullptr),
      take_screen_shot_(false) {}

////////////////////////////////////////////////////////////////////////////////

WindowBase::~WindowBase() {
  destroy_context();
}

void WindowBase::destroy_context() {
  warpRR_ = nullptr;
  warpGR_ = nullptr;
  warpBR_ = nullptr;
  warpRL_ = nullptr;
  warpGL_ = nullptr;
  warpBL_ = nullptr;

  blend_state_.reset();
  depth_stencil_state_.reset();
  fullscreen_quad_.reset();
  fullscreen_shader_.program_.reset();

  ctx_.render_pipelines.clear();
  ctx_.render_context.reset();
  //ctx_.display.reset();
  ctx_.render_device.reset();
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::init_context() {
  if (config.get_warp_matrix_red_right() == "" ||
      config.get_warp_matrix_green_right() == "" ||
      config.get_warp_matrix_blue_right() == "" ||
      config.get_warp_matrix_red_left() == "" ||
      config.get_warp_matrix_green_left() == "" ||
      config.get_warp_matrix_blue_left() == "") {

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    fullscreen_shader_.create_from_sources(
      factory.read_shader_file("resources/shaders/display_shader.vert"),
      factory.read_shader_file("resources/shaders/display_shader.frag"));
#else
    fullscreen_shader_.create_from_sources(
      Resources::lookup_shader(Resources::shaders_display_shader_vert),
      Resources::lookup_shader(Resources::shaders_display_shader_frag));
#endif
  } else {
    warpRR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_red_right());

    warpGR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_green_right());

    warpBR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_blue_right());

    warpRL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_red_left());

    warpGL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_green_left());

    warpBL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_blue_left());

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    fullscreen_shader_.create_from_sources(
      factory.read_shader_file("resources/shaders/display_shader.vert"),
      factory.read_shader_file("resources/shaders/display_shader_warped.frag"));
#else
    fullscreen_shader_.create_from_sources(
      Resources::lookup_shader(Resources::shaders_display_shader_vert),
      Resources::lookup_shader(Resources::shaders_display_shader_warped_frag)
      );
#endif
  }

  ctx_.render_device  = scm::gl::render_device_ptr(new scm::gl::render_device());
  ctx_.render_context = ctx_.render_device->main_context();

  {
    std::lock_guard<std::mutex> lock(last_context_id_mutex_);
    ctx_.id = last_context_id_++;
  }

  ctx_.render_window = this;

  fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
    ctx_.render_device, scm::math::vec2f(-1.f, -1.f), scm::math::vec2f(1.f, 1.f)));

  depth_stencil_state_ = ctx_.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

  blend_state_ = ctx_.render_device->create_blend_state(true,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE);
  if (config.get_debug()) {
    ctx_.render_context->register_debug_callback(boost::make_shared<DebugOutput>());
  }
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::start_frame() const {
  ctx_.render_context->clear_default_color_buffer(
      scm::gl::FRAMEBUFFER_BACK, scm::math::vec4f(0.f, 0.f, 0.f, 1.0f));

  ctx_.render_context->clear_default_depth_stencil_buffer();
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::take_screen_shot() {
  std::lock_guard<std::mutex> lock(screen_shot_mutex_);
  take_screen_shot_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(std::shared_ptr<Texture> const& center_texture) {

  display(center_texture, true);

  if (config.get_stereo_mode() != StereoMode::MONO) {
    display(center_texture, false);
  }
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(std::shared_ptr<Texture> const& texture,
                         bool is_left) {

  switch (config.get_stereo_mode()) {
    case StereoMode::MONO:
    case StereoMode::SIDE_BY_SIDE:
      display(texture,
              is_left ? config.get_left_resolution() : config.get_right_resolution(),
              is_left ? config.get_left_position() : config.get_right_position(),
              WindowBase::FULL, is_left, true);
      break;
    case StereoMode::ANAGLYPH_RED_CYAN:
      display(texture,
              is_left ? config.get_left_resolution() : config.get_right_resolution(),
              is_left ? config.get_left_position() : config.get_right_position(),
              is_left ? WindowBase::RED : WindowBase::CYAN,
              is_left, is_left);
      break;
    case StereoMode::ANAGLYPH_RED_GREEN:
      display(texture,
              is_left ? config.get_left_resolution() : config.get_right_resolution(),
              is_left ? config.get_left_position() : config.get_right_position(),
              is_left ? WindowBase::RED : WindowBase::GREEN,
              is_left, is_left);
      break;
    case StereoMode::CHECKERBOARD:
      display(texture,
              is_left ? config.get_left_resolution() : config.get_right_resolution(),
              is_left ? config.get_left_position() : config.get_right_position(),
              is_left ? WindowBase::CHECKER_EVEN : WindowBase::CHECKER_ODD,
              is_left, true);
      break;
  }

  std::lock_guard<std::mutex> lock(screen_shot_mutex_);
  if (take_screen_shot_) {
    take_screen_shot_ = false;

    auto texture_ptr(std::dynamic_pointer_cast<Texture2D>(texture));

    if (texture_ptr) {
      auto texture_2d_ptr(boost::dynamic_pointer_cast<scm::gl::texture_2d>(texture_ptr->get_buffer(ctx_)));
      auto width = texture_2d_ptr->descriptor()._size.x;
      auto height = texture_2d_ptr->descriptor()._size.y;
      // auto bpp = scm::gl::bit_per_pixel(texture_2d_ptr->format());
      // auto gl_type = scm::gl::util::gl_base_type(texture_2d_ptr->format());
      // auto gl_internal_format = scm::gl::util::gl_internal_format(texture_2d_ptr->format());
      // auto gl_base_format = scm::gl::util::gl_base_format(texture_2d_ptr->format());

      int size = width * height * scm::gl::size_of_format(texture_2d_ptr->format());
      std::vector<char> data(size);

      ctx_.render_context->retrieve_texture_data(texture_2d_ptr, 0, data.data());

    }
  }

}


////////////////////////////////////////////////////////////////////////////////

RenderContext* WindowBase::get_context() { return &ctx_; }

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(std::shared_ptr<Texture> const& texture,
                     math::vec2ui const& size,
                     math::vec2ui const& position,
                     TextureDisplayMode mode,
                     bool is_left,
                     bool clear) {

  fullscreen_shader_.use(ctx_);
  fullscreen_shader_.set_uniform(ctx_, texture->get_handle(ctx_), "sampler");

  if (is_left) {
    if (warpRL_) fullscreen_shader_.set_uniform(ctx_, warpRL_->get_handle(ctx_), "warpR");
    if (warpGL_) fullscreen_shader_.set_uniform(ctx_, warpGL_->get_handle(ctx_), "warpG");
    if (warpBL_) fullscreen_shader_.set_uniform(ctx_, warpBL_->get_handle(ctx_), "warpB");
  } else {
    if (warpRR_) fullscreen_shader_.set_uniform(ctx_, warpRR_->get_handle(ctx_), "warpR");
    if (warpGR_) fullscreen_shader_.set_uniform(ctx_, warpGR_->get_handle(ctx_), "warpG");
    if (warpBR_) fullscreen_shader_.set_uniform(ctx_, warpBR_->get_handle(ctx_), "warpB");
  }

  std::string subroutine = subroutine_from_mode(mode);

  fullscreen_shader_.set_subroutine(
      ctx_, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", subroutine);

  ctx_.render_context->set_viewport(scm::gl::viewport(position, size));
  ctx_.render_context->set_depth_stencil_state(depth_stencil_state_);

  if (!clear) {
    ctx_.render_context->set_blend_state(blend_state_);
  }

  fullscreen_quad_->draw(ctx_.render_context);

  ctx_.render_context->reset_state_objects();
  fullscreen_shader_.unuse(ctx_);
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::DebugOutput::operator()(scm::gl::debug_source source,
                                     scm::gl::debug_type type,
                                     scm::gl::debug_severity severity,
                                     const std::string& message) const {

  Logger::LOG_MESSAGE << "[Source: " << scm::gl::debug_source_string(source)
                      << ", type: " << scm::gl::debug_type_string(type)
                      << ", severity: " << scm::gl::debug_severity_string(severity)
                      << "]: " << message << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

}
