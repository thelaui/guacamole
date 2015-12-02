#include "SteepestDescentOptimizer.hpp"

#include <iostream>

#include <scm/gl_core/math.h>

#include <unistd.h>
#include <fstream>

#include <texture_stream/texture_stream.hpp>

void SteepestDescentOptimizer::run(scm::math::mat4d& optimal_transform, scm::math::mat4d& optimal_difference) {

  int iteration_count(0);
  bool optimum_reached(false);
  current_step_length_ = 1.0;

  auto current_transform = initial_transform;
  auto current_difference = scm::math::mat4d::identity();

  current_photo_ = retrieve_photo();

  while (!optimum_reached && ++iteration_count <= 100) { // && current_step_length_ != 0.0) {
    auto gradient(get_gradient(current_transform));

    optimum_reached = gradient == scm::math::mat<double, 6, 1>();

    if (!optimum_reached) {
      // update step length
      update_step_length(current_transform, gradient);
      // current_step_length_ = 0.5;

      // negative gradient is always a descent direction

      scm::math::mat4d new_translation(scm::math::make_translation(
        -gradient.data_array[0] * current_step_length_,
        -gradient.data_array[1] * current_step_length_,
        -gradient.data_array[2] * current_step_length_
      ));

      scm::math::mat4d new_rot_x(scm::math::make_rotation(
        -gradient.data_array[3] * current_step_length_,
        1.0, 0.0, 0.0
      ));

      scm::math::mat4d new_rot_y(scm::math::make_rotation(
        -gradient.data_array[4] * current_step_length_,
        0.0, 1.0, 0.0
      ));

      scm::math::mat4d new_rot_z(scm::math::make_rotation(
        -gradient.data_array[5] * current_step_length_,
        0.0, 0.0, 1.0
      ));

      current_difference = new_translation * new_rot_z *  new_rot_x *  new_rot_y;
      current_transform = current_transform * current_difference;
      optimal_difference *= current_difference;
    } else {
      std::cout << "Reached optimum after " << iteration_count << " iterations." << std::endl;
    }
  }

  if (!optimum_reached) {
    std::cout << "Aborted optimization: Maximum iteration count reached. " << iteration_count << std::endl;
  }

  optimal_transform = current_transform;

}

////////////////////////////////////////////////////////////////////////////////

scm::math::mat<double, 6, 1> SteepestDescentOptimizer::get_gradient(
                                        scm::math::mat4d const& central_transform) const {

  scm::math::mat<double, 6, 1> gradient;

  // const double position_offset(0.01);
  // const double rotation_offset(0.5);

  const double position_offset(0.01 * current_step_length_);
  const double rotation_offset(0.5 * current_step_length_);

  cv::Mat screen_shot;

  auto offset_mat_left(scm::math::mat4d::identity());
  auto offset_mat_right(scm::math::mat4d::identity());
  double left_error(0.0);
  double right_error(0.0);

  // calculate gradient in x direction
  offset_mat_left = scm::math::make_translation(-position_offset, 0.0, 0.0);
  offset_mat_right = scm::math::make_translation(position_offset, 0.0, 0.0);

  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient.data_array[0] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in y direction
  offset_mat_left = scm::math::make_translation(0.0, -position_offset, 0.0);
  offset_mat_right = scm::math::make_translation(0.0, position_offset, 0.0);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient.data_array[1] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in z direction
  offset_mat_left = scm::math::make_translation(0.0, 0.0, -position_offset);
  offset_mat_right = scm::math::make_translation(0.0, 0.0, position_offset);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient.data_array[2] = (right_error - left_error) /
                           (2 * position_offset);

  // calculate gradient in x rotation
  offset_mat_left = scm::math::make_rotation(-rotation_offset, 1.0, 0.0, 0.0);
  offset_mat_right = scm::math::make_rotation(rotation_offset, 1.0, 0.0, 0.0);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient.data_array[3] = (right_error - left_error) /
                           (2 * rotation_offset);

  // calculate gradient in y rotation
  offset_mat_left = scm::math::make_rotation(-rotation_offset, 0.0, 1.0, 0.0);
  offset_mat_right = scm::math::make_rotation(rotation_offset, 0.0, 1.0, 0.0);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient.data_array[4] = (right_error - left_error) /
                           (2 * rotation_offset);

  // calculate gradient in z rotation
  offset_mat_left = scm::math::make_rotation(-rotation_offset, 0.0, 0.0, 1.0);
  offset_mat_right = scm::math::make_rotation(rotation_offset, 0.0, 0.0, 1.0);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient.data_array[5] = (right_error - left_error) /
                           (2 * rotation_offset);


  double gradient_length(0.0);

  for (unsigned int i(0); i < 6; ++i) {
    gradient_length += std::pow(gradient.data_array[i], 2);
  }

  gradient_length = std::sqrt(gradient_length);

  if (gradient_length <= 0.001) {
    return scm::math::mat<double, 6, 1>();
  }

  return gradient;
}

////////////////////////////////////////////////////////////////////////////////

void SteepestDescentOptimizer::update_step_length(
                             scm::math::mat4d const& central_transform,
                             scm::math::mat<double, 6, 1> const& gradient) {

  // calculate optimal step length according to Armijo rule

  int iteration_count(0);
  double beta(0.9);
  double epsilon(0.01);
  double previous_step_length(current_step_length_);
  current_step_length_ = 1.0;

  auto current_transform(central_transform);

  // phi(0)
  cv::Mat screen_shot(retrieve_screen_shot(central_transform));
  auto central_error(error_function(current_photo_, screen_shot));

  texstr::StringUtils::set_high_precision(std::cout);

  while (true) {
    scm::math::mat4d new_translation(scm::math::make_translation(
      -gradient.data_array[0] * current_step_length_,
      -gradient.data_array[1] * current_step_length_,
      -gradient.data_array[2] * current_step_length_
    ));

    scm::math::mat4d new_rot_x(scm::math::make_rotation(
      -gradient.data_array[3] * current_step_length_,
      1.0, 0.0, 0.0
    ));

    scm::math::mat4d new_rot_y(scm::math::make_rotation(
      -gradient.data_array[4] * current_step_length_,
      0.0, 1.0, 0.0
    ));

    scm::math::mat4d new_rot_z(scm::math::make_rotation(
      -gradient.data_array[5] * current_step_length_,
      0.0, 0.0, 1.0
    ));

    current_transform = central_transform * new_translation * new_rot_z *  new_rot_x *  new_rot_y;;

    // phi(t_k)
    screen_shot = retrieve_screen_shot(current_transform);
    auto current_error(error_function(current_photo_, screen_shot));

    // phi'(0)
    double phi_derivative(0.0);
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

  // if (current_step_length_ == previous_step_length && current_step_length_ != 1.0)  {
  //   current_step_length_ = 0.0;
  // }

  std::cout << "optimal_step_length: " << current_step_length_ << std::endl;

}







