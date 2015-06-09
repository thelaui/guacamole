#include "Navigator.hpp"

#include <scm/gl_core/math.h>
#include <gua/math/math.hpp>
#include <GL/freeglut.h>
#include <chrono>
#include <iostream>

Navigator::Navigator()
  : transform_(scm::math::mat4f::identity())
  , current_location_(scm::math::vec4f(0.0))
  , current_rotation_(scm::math::vec2f(0.0))
  , mouse_position_(scm::math::vec2i(0))
  , mouse_movement_(scm::math::vec2i(0))
  , w_pressed_(false)
  , s_pressed_(false)
  , a_pressed_(false)
  , d_pressed_(false)
  , mlb_pressed_(false)
  , frame_time_(-1.0)
{}

void Navigator::update() {
  if (frame_time_ == -1.0) {
    timer_.start();
    frame_time_ = 0.0;
  } else {

    frame_time_ = timer_.get_elapsed();

    const float rotation_speed = 0.25f;
    const float motion_speed = 1.f;
    // const float motion_speed = 0.01f;

    auto y_rot(scm::math::mat4f::identity());
    auto x_rot(scm::math::mat4f::identity());

    if (mlb_pressed_) {
      current_rotation_ -= mouse_movement_;
    }

    y_rot = scm::math::make_rotation( current_rotation_.x, 0.f, 1.f, 0.f);
    x_rot = scm::math::make_rotation(-current_rotation_.y, 1.f, 0.f, 0.f);

    auto rotation = y_rot * x_rot;

    if (w_pressed_) {
      current_location_ += (rotation * scm::math::make_translation(0.f, 0.f, -motion_speed))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    if (s_pressed_) {
      current_location_ += (rotation * scm::math::make_translation(0.f, 0.f, motion_speed))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    if (a_pressed_) {
      current_location_ += (rotation * scm::math::make_translation(-motion_speed, 0.f, 0.f))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    if (d_pressed_) {
      current_location_ += (rotation * scm::math::make_translation(motion_speed, 0.f, 0.f))
                           * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
    }

    auto target = scm::math::make_translation(current_location_.x, current_location_.y, current_location_.z) * rotation;

    auto orig_rot = scm::math::quat<float>::from_matrix(transform_);
    auto orig   = scm::math::make_translation(transform_[12], transform_[13], transform_[14]) *
                  orig_rot.to_matrix();

    float smoothness = frame_time_ * 10.0;

    // transform_ = orig * (1.f - smoothness) + target * smoothness;
    transform_ = target;

    mouse_movement_ = scm::math::vec2i(0);

    w_pressed_ = false;
    s_pressed_ = false;
    a_pressed_ = false;
    d_pressed_ = false;
    timer_.reset();
  }
}

void Navigator::set_transform(scm::math::mat4f const& transform) {
  transform_ = transform;
  current_location_ = scm::math::vec4f(transform_[12], transform_[13], transform_[14], 1.0f);

  auto rotation(gua::math::get_euler_angles(gua::math::mat4(transform)));
  current_rotation_ = scm::math::vec2f(rotation.y, -rotation.x);
}

scm::math::mat4f const& Navigator::get_transform() const {
  return transform_;
}

void Navigator::set_key_press(unsigned char key) {
  switch (key) {
    case 'w':
      w_pressed_ = true;
      break;

    case 's':
      s_pressed_ = true;
      break;

    case 'a':
      a_pressed_ = true;
      break;

    case 'd':
      d_pressed_ = true;
      break;

    default: break;
  }
}

void Navigator::set_mouse_button(int button, int state) {
  switch (button) {
    case 0:
      mlb_pressed_ = state == 1;
      break;
    default: break;
  }
}

void Navigator::set_mouse_position(scm::math::vec2i const& new_position) {
  mouse_movement_ = new_position - mouse_position_;
  mouse_position_ = new_position;
}

