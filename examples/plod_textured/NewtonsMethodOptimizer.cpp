#include "NewtonsMethodOptimizer.hpp"

#include <iostream>

#include <scm/gl_core/math.h>

#include <unistd.h>
#include <fstream>

void NewtonsMethodOptimizer::run(scm::math::mat4f& optimal_transform, scm::math::mat4f& optimal_difference) {

  float step_length(0.5f);
  int iteration_count(0);
  bool optimum_reached(false);

  auto current_transform = initial_transform;
  auto current_difference = scm::math::mat4f::identity();

  while (!optimum_reached && iteration_count < 1000) {
    ++iteration_count;
    auto descent_direction(get_descent_direction(current_transform));

    optimum_reached = descent_direction == scm::math::mat<float, 6, 1>();

    if (!optimum_reached) {
      scm::math::mat4f new_translation(scm::math::make_translation(
        descent_direction.data_array[0] * step_length,
        descent_direction.data_array[1] * step_length,
        descent_direction.data_array[2] * step_length
      ));

      scm::math::mat4f new_rot_x(scm::math::make_rotation(
        descent_direction.data_array[3] * step_length,
        1.f, 0.f, 0.f
      ));

      scm::math::mat4f new_rot_y(scm::math::make_rotation(
        descent_direction.data_array[4] * step_length,
        0.f, 1.f, 0.f
      ));

      scm::math::mat4f new_rot_z(scm::math::make_rotation(
        descent_direction.data_array[5] * step_length,
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

scm::math::mat<float, 6, 1> NewtonsMethodOptimizer::get_descent_direction(
                                        scm::math::mat4f const& central_transform) const {

  scm::math::mat<float, 6, 1> descent_direction;
  scm::math::mat<float, 6, 1> gradient;
  scm::math::mat<float, 6, 6> hessian;

  const float position_offset(0.01f);
  const float rotation_offset(0.5f);

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

  bool is_extremum(true);

  for (unsigned int i(0); i < 6; ++i) {
    if (std::abs(gradient.data_array[i]) >= 0.01) {
      is_extremum = false;
      break;
    }
  }

  if (is_extremum) {
    return scm::math::mat<float, 6, 1>();
  }


  return descent_direction;
}

































