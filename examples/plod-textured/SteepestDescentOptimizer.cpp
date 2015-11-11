#include "SteepestDescentOptimizer.hpp"

#include <iostream>

#include <scm/gl_core/math.h>

#include <unistd.h>
#include <fstream>

void SteepestDescentOptimizer::run(scm::math::mat4f& optimal_transform, scm::math::mat4f& optimal_difference) {

  int iteration_count(0);
  bool optimum_reached(false);

  auto current_transform = initial_transform;
  auto current_difference = scm::math::mat4f::identity();

  while (!optimum_reached && iteration_count < 1000) {
    ++iteration_count;
    auto gradient(get_gradient(current_transform));

    optimum_reached = gradient == scm::math::mat<float, 6, 1>();

    if (!optimum_reached) {
      // update step length
      update_step_length(current_transform, gradient);
      // float step_length = 0.5f;

      // negative gradient is always a descent direction

      scm::math::mat4f new_translation(scm::math::make_translation(
        -gradient.data_array[0] * current_step_length_,
        -gradient.data_array[1] * current_step_length_,
        -gradient.data_array[2] * current_step_length_
      ));

      scm::math::mat4f new_rot_x(scm::math::make_rotation(
        -gradient.data_array[3] * current_step_length_,
        1.f, 0.f, 0.f
      ));

      scm::math::mat4f new_rot_y(scm::math::make_rotation(
        -gradient.data_array[4] * current_step_length_,
        0.f, 1.f, 0.f
      ));

      scm::math::mat4f new_rot_z(scm::math::make_rotation(
        -gradient.data_array[5] * current_step_length_,
        0.f, 0.f, 1.f
      ));

      current_difference = new_translation * new_rot_y * new_rot_x * new_rot_z;
      current_transform = current_transform * current_difference;
      optimal_difference *= current_difference;
    } else {
      std::cout << "Reached optimum after " << iteration_count << " iterations." << std::endl;
    }
  }

  if (!optimum_reached) {
    std::cout << "Aborted optimization: Maximum iteration count reached." << std::endl;
  }

  optimal_transform = current_transform;

}

////////////////////////////////////////////////////////////////////////////////

scm::math::mat<float, 6, 1> SteepestDescentOptimizer::get_gradient(
                                        scm::math::mat4f const& central_transform) const {

  scm::math::mat<float, 6, 1> gradient;

  const float position_offset(0.01f * current_step_length_);
  const float rotation_offset(0.5f * current_step_length_);

  auto offset_mat_left(scm::math::mat4f::identity());
  auto offset_mat_right(scm::math::mat4f::identity());
  float left_error(0.f);
  float right_error(0.f);

  // calculate gradient in x direction
  offset_mat_left = scm::math::make_translation(-position_offset, 0.f, 0.f);
  offset_mat_right = scm::math::make_translation(position_offset, 0.f, 0.f);
  left_error = error_function(central_transform * offset_mat_left);
  right_error = error_function(central_transform * offset_mat_right);

  gradient.data_array[0] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in y direction
  offset_mat_left = scm::math::make_translation(0.f, -position_offset, 0.f);
  offset_mat_right = scm::math::make_translation(0.f, position_offset, 0.f);
  left_error = error_function(central_transform * offset_mat_left);
  right_error = error_function(central_transform * offset_mat_right);

  gradient.data_array[1] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in z direction
  offset_mat_left = scm::math::make_translation(0.f, 0.f, -position_offset);
  offset_mat_right = scm::math::make_translation(0.f, 0.f, position_offset);
  left_error = error_function(central_transform * offset_mat_left);
  right_error = error_function(central_transform * offset_mat_right);

  gradient.data_array[2] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in x rotation
  offset_mat_left = scm::math::make_rotation(-rotation_offset, 1.f, 0.f, 0.f);
  offset_mat_right = scm::math::make_rotation(rotation_offset, 1.f, 0.f, 0.f);
  left_error = error_function(central_transform * offset_mat_left);
  right_error = error_function(central_transform * offset_mat_right);

  gradient.data_array[3] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in y rotation
  offset_mat_left = scm::math::make_rotation(-rotation_offset, 0.f, 1.f, 0.f);
  offset_mat_right = scm::math::make_rotation(rotation_offset, 0.f, 1.f, 0.f);
  left_error = error_function(central_transform * offset_mat_left);
  right_error = error_function(central_transform * offset_mat_right);

  gradient.data_array[4] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in z rotation
  offset_mat_left = scm::math::make_rotation(-rotation_offset, 0.f, 0.f, 1.f);
  offset_mat_right = scm::math::make_rotation(rotation_offset, 0.f, 0.f, 1.f);
  left_error = error_function(central_transform * offset_mat_left);
  right_error = error_function(central_transform * offset_mat_right);

  gradient.data_array[5] = (right_error - left_error) /
                           (2 * position_offset);


  float gradient_length(0.f);

  for (unsigned int i(0); i < 6; ++i) {
    gradient_length += std::pow(gradient.data_array[i], 2);
  }

  gradient_length = std::sqrt(gradient_length);

  if (gradient_length <= 0.05f) {
    return scm::math::mat<float, 6, 1>();
  }

  return gradient;
}

////////////////////////////////////////////////////////////////////////////////

void SteepestDescentOptimizer::update_step_length(
                             scm::math::mat4f const& central_transform,
                             scm::math::mat<float, 6, 1> const& gradient) {

  // calculate optimal step length according to Armijo rule

  int iteration_count(0);
  float beta(0.9);
  float epsilon(0.01);
  current_step_length_ = 1.f;
  float optimal_step_length(current_step_length_);

  auto current_transform(central_transform);

  // phi(0)
  auto central_error(error_function(central_transform));

  while (++iteration_count <= 100 && current_step_length_ >= 0.1) {
    scm::math::mat4f new_translation(scm::math::make_translation(
      -gradient.data_array[0] * current_step_length_,
      -gradient.data_array[1] * current_step_length_,
      -gradient.data_array[2] * current_step_length_
    ));

    scm::math::mat4f new_rot_x(scm::math::make_rotation(
      -gradient.data_array[3] * current_step_length_,
      1.f, 0.f, 0.f
    ));

    scm::math::mat4f new_rot_y(scm::math::make_rotation(
      -gradient.data_array[4] * current_step_length_,
      0.f, 1.f, 0.f
    ));

    scm::math::mat4f new_rot_z(scm::math::make_rotation(
      -gradient.data_array[5] * current_step_length_,
      0.f, 0.f, 1.f
    ));

    current_transform = current_transform * new_translation * new_rot_y * new_rot_x * new_rot_z;

    // phi(t_k)
    auto current_error(error_function(current_transform));

    // phi'(0)
    float phi_derivative(0.f);
    for (int i(0); i < 6; ++i) {
      phi_derivative += gradient.data_array[i] * (-gradient.data_array[i]);
    }

    std::cout << "current_step_length_: " << current_step_length_ << std::endl;
    std::cout << "central_error: " << central_error << std::endl;
    std::cout << "current_error: " << current_error << std::endl;

    if (current_error <= central_error + epsilon * current_step_length_ * phi_derivative) {
      break;
    } else {
      current_step_length_ *= beta;
    }
  }

  std::cout << "optimal_step_length: " << current_step_length_ << std::endl;

}







