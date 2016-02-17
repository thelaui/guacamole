#include "SteepestDescentOptimizer.hpp"

#include <iostream>

#include <scm/gl_core/math.h>
#include <gua/math.hpp>

#include <unistd.h>
#include <fstream>

#include <texture_stream/texture_stream.hpp>

#include "ImageClusterGenerator.hpp"

////////////////////////////////////////////////////////////////////////////////

bool SteepestDescentOptimizer::run(scm::math::mat4d& optimal_transform,
                                   scm::math::mat4d& optimal_difference) {

  bool success(false);
  int iteration_count(0);
  bool optimum_reached(false);
  current_step_length_ = 1.0;

  optimal_transform = initial_transform;

  auto current_transform = initial_transform;
  auto current_difference = scm::math::mat4d::identity();
  current_photo_ = retrieve_photo();

  auto screen_shot(retrieve_screen_shot(current_transform));
  ImageClusterGenerator::instance()->init(current_photo_, screen_shot, 3);

  if (pre_classification(current_photo_, screen_shot)) {
    std::cout << "Pre-Classification: Image accepted." << std::endl;

    while (!optimum_reached && ++iteration_count <= 100) { // && current_step_length_ != 0.0) {
      auto gradient(get_gradient(current_transform));
      // std::cout << "Gradient: " << gradient << std::endl;
      optimum_reached = gradient == scm::math::mat<double, 6, 1>();

      if (!optimum_reached) {
        // update step length
        update_step_length(current_transform, gradient);

        if (current_step_length_ == 0.0) {
          optimum_reached = true;
          std::cout << "Found smallest step length." << std::endl;

        } else {
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
        }
      }
    }

    if (!optimum_reached) {
      std::cout << "Aborted optimization: Maximum iteration count reached. " << iteration_count << std::endl;
    } else {
      std::cout << "Reached optimum after " << iteration_count << " iterations." << std::endl;
    }

    if (post_classification(current_transform)) {
      std::cout << "Post-Classification: Image accepted!" << std::endl;
      optimal_transform = current_transform;
      success = true;
    } else {
      std::cout << "Post-Classification: Image rejected!" << std::endl;
      optimal_transform = initial_transform;
      optimal_difference = scm::math::mat4d::identity();
    }

  } else {
    std::cout << "Pre-Classification: Image rejected!" << std::endl;
  }

  return success;
}

////////////////////////////////////////////////////////////////////////////////

void SteepestDescentOptimizer::run_round_robin(
                                         scm::math::mat4d& optimal_transform,
                                         scm::math::mat4d& optimal_difference) {

  int iteration_count(0);
  bool optimum_reached(false);
  current_step_length_ = 1.0;

  auto current_transform = initial_transform;
  auto current_difference = scm::math::mat4d::identity();

  current_photo_ = retrieve_photo();

  auto screen_shot(retrieve_screen_shot(current_transform));

  ImageClusterGenerator::instance()->init(current_photo_, screen_shot, 3);

  if (pre_classification(current_photo_, screen_shot)) {
    std::cout << "Pre-Classification: Image accepted." << std::endl;

    auto initial_translation(gua::math::get_translation(initial_transform));

    std::vector<std::vector<double>> positions_per_dimension(6);

    const std::vector<std::string> dimension_names(
      {"trans_x", "trans_y", "trans_z", "rot_x", "rot_y", "rot_z"}
    );

    while (!optimum_reached && ++iteration_count <= 50) {
      int optimal_dimension_count(0);

      for (int dimension(0); dimension < 6; ++dimension) {

        double gradient(get_gradient_for_dimension(current_transform, dimension));
        std::cout << "Gradient for " << dimension_names[dimension] << ": " << gradient << std::endl;

        if (std::abs(gradient) <= 0.001) {
          ++optimal_dimension_count;
          std::cout << "Found optimum for " << dimension_names[dimension] << "." << std::endl;
        } else {

          update_step_length_for_dimension(current_transform, gradient, dimension);

          if (current_step_length_ == 0.0) {
            ++optimal_dimension_count;
            std::cout << "Found smallest step length for " << dimension_names[dimension] << "." << std::endl;
          } else {

            scm::math::mat4d new_translation(scm::math::mat4d::identity());
            scm::math::mat4d new_rotation(scm::math::mat4d::identity());

            if (dimension < 3) {
              // update translation
              scm::math::vec3d translation_vector(0.0);
              translation_vector[dimension] = -gradient * current_step_length_;

              new_translation = scm::math::make_translation(translation_vector);
            } else {
              // update rotation
              scm::math::vec3d rotation_axis_vector(0.0);
              rotation_axis_vector[dimension - 3] = 1.0;

              new_rotation = scm::math::make_rotation(-gradient * current_step_length_,
                                                      rotation_axis_vector);
            }


            current_difference = new_translation * new_rotation;
            current_transform = current_transform * current_difference;
            optimal_difference *= current_difference;

            // auto current_translation(gua::math::get_translation(current_transform));
            // positions_per_dimension[dimension].push_back(
            //   current_translation[dimension] - initial_translation[dimension]
            // );

          }
        }

      }

      optimum_reached = optimal_dimension_count == 6;

    }

    if (!optimum_reached) {
      std::cout << "Aborted optimization: Maximum iteration count reached. " << iteration_count << std::endl;
    } else {
      std::cout << "Reached optimum after " << iteration_count << " iterations." << std::endl;
    }

    // std::cout << "Travelled positions" << std::endl;
    // for (int dimension(0); dimension < 6; ++dimension) {
    //   std::cout << dimension_names[dimension] << std::endl;
    //   for (int step(0); step < positions_per_dimension[dimension].size(); ++step) {
    //     std::cout << step + 1 << " " << positions_per_dimension[dimension][step] << std::endl;
    //   }
    // }

  } else {
    std::cout << "Pre-Classification: Image rejected!" << std::endl;
  }

  optimal_transform = current_transform;

}

////////////////////////////////////////////////////////////////////////////////

scm::math::mat<double, 6, 1> SteepestDescentOptimizer::get_gradient(
                                        scm::math::mat4d const& central_transform) const {

  scm::math::mat<double, 6, 1> gradient;

  for (int dimension(0); dimension < 6; ++dimension) {
    gradient.data_array[dimension] = get_gradient_for_dimension(central_transform, dimension);
  }

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

double SteepestDescentOptimizer::get_gradient_for_dimension(
                                      scm::math::mat4d const& central_transform,
                                      int dimension) const {

  const double position_offset(0.01);// * current_step_length_);
  const double rotation_offset(0.5);// * current_step_length_);


  auto offset_mat_left(scm::math::mat4d::identity());
  auto offset_mat_right(scm::math::mat4d::identity());
  double left_error(0.0);
  double right_error(0.0);

  double gradient(0.0);
  double lookup_distance(0.0);

  if (dimension < 3) {
    // gradient for change in position

    scm::math::vec3d translation_vector(0.0);
    translation_vector[dimension] = position_offset;

    offset_mat_left = scm::math::make_translation(-translation_vector);
    offset_mat_right = scm::math::make_translation(translation_vector);

    lookup_distance = 2 * position_offset;

  } else if (dimension < 6) {
    // gradient for change in rotation

    scm::math::vec3d rotation_axis_vector(0.0);
    rotation_axis_vector[dimension - 3] = 1.0;

    offset_mat_left = scm::math::make_rotation(-rotation_offset, rotation_axis_vector);
    offset_mat_right = scm::math::make_rotation(rotation_offset, rotation_axis_vector);

    lookup_distance = 2 * rotation_offset;
  }

  cv::Mat screen_shot;
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_left);
  left_error = error_function(current_photo_, screen_shot);
  screen_shot = retrieve_screen_shot(central_transform * offset_mat_right);
  right_error = error_function(current_photo_, screen_shot);

  gradient = (right_error - left_error) / lookup_distance;

  return gradient;
}

////////////////////////////////////////////////////////////////////////////////

void SteepestDescentOptimizer::update_step_length(
                             scm::math::mat4d const& central_transform,
                             scm::math::mat<double, 6, 1> const& gradient) {

  // calculate optimal step length according to Armijo rule

  double beta(0.9);
  double epsilon(0.01);

  current_step_length_ = 1.0;

  auto current_transform(central_transform);

  // phi(0)
  cv::Mat screen_shot(retrieve_screen_shot(central_transform));
  auto central_error(error_function(current_photo_, screen_shot));

  texstr::StringUtils::set_high_precision(std::cout);

  // phi'(0)
  double phi_derivative(0.0);
  for (int i(0); i < 6; ++i) {
    phi_derivative += gradient.data_array[i] * (-gradient.data_array[i]);
  }

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

    cv::Mat screen_shot_bak(retrieve_screen_shot(current_transform));

    current_transform = central_transform * new_translation * new_rot_z *  new_rot_x *  new_rot_y;

    // phi(t_k)
    screen_shot = retrieve_screen_shot(current_transform);
    auto current_error(error_function(current_photo_, screen_shot));

    cv::absdiff(screen_shot, screen_shot_bak, screen_shot_bak);
    auto screen_shot_error(cv::sum(screen_shot_bak)[0]);
    if (screen_shot_error == 0.0) {

      current_step_length_ = 0.0;
      break;
    }

    // std::cout << "phi_derivative: " << phi_derivative << std::endl;
    // std::cout << "central_error: " << central_error << std::endl;
    // std::cout << "current_error: " << current_error << std::endl;

    if (current_error <= central_error + epsilon * current_step_length_ * phi_derivative) {
      break;
    } else {
      current_step_length_ *= beta;
    }
  }

  // std::cout << "optimal_step_length: " << current_step_length_ << std::endl;

}

////////////////////////////////////////////////////////////////////////////////

void SteepestDescentOptimizer::update_step_length_for_dimension(
                             scm::math::mat4d const& central_transform,
                             double gradient, int dimension) {

  // calculate optimal step length according to Armijo rule

  double beta(0.9);
  double epsilon(0.01);

  if (dimension < 3) {
    // initial step length for translations
    current_step_length_ = 1.0;
  } else if (dimension < 6) {
    // initial step length for rotations
    current_step_length_ = 1.0;
  }

  auto current_transform(central_transform);

  // phi(0)
  cv::Mat screen_shot(retrieve_screen_shot(central_transform));
  auto central_error(error_function(current_photo_, screen_shot));

  texstr::StringUtils::set_high_precision(std::cout);

  // phi'(0)
  double phi_derivative(gradient * -gradient);

  while (true) {
    scm::math::mat4d new_translation(scm::math::mat4d::identity());
    scm::math::mat4d new_rotation(scm::math::mat4d::identity());

    if (dimension < 3) {
      scm::math::vec3d translation_vector(0.0);
      translation_vector[dimension] = -gradient * current_step_length_;

      new_translation = scm::math::make_translation(translation_vector);
    } else if (dimension < 6) {
      scm::math::vec3d rotation_axis_vector(0.0);
      rotation_axis_vector[dimension - 3] = 1.0;

      new_rotation = scm::math::make_rotation(-gradient * current_step_length_,
                                               rotation_axis_vector);
    }

    cv::Mat screen_shot_bak(retrieve_screen_shot(current_transform));

    current_transform = central_transform * new_translation * new_rotation;

    // phi(t_k)
    screen_shot = retrieve_screen_shot(current_transform);
    auto current_error(error_function(current_photo_, screen_shot));

    cv::absdiff(screen_shot, screen_shot_bak, screen_shot_bak);
    auto screen_shot_error(cv::sum(screen_shot_bak)[0]);
    if (screen_shot_error == 0.0) {

      current_step_length_ = 0.0;
      break;
    }

    // std::cout << "phi_derivative: " << phi_derivative << std::endl;
    // std::cout << "central_error: " << central_error << std::endl;
    // std::cout << "current_error: " << current_error << std::endl;

    if (current_error <= central_error + epsilon * current_step_length_ * phi_derivative) {
      break;
    } else {
      current_step_length_ *= beta;
    }
  }

  // std::cout << "optimal_step_length: " << current_step_length_ << std::endl;

}

////////////////////////////////////////////////////////////////////////////////

bool SteepestDescentOptimizer::post_classification(scm::math::mat4d const& optimal_transform) const {

  scm::math::mat4d optimization_offset(scm::math::inverse(initial_transform) * optimal_transform);

  bool accepted(true);

  // check if unreasonably large jump occured
  for (int row(0); row < 3; ++row) {

    if(std::abs(optimization_offset.column(3)[row]) > 0.5) {
      accepted = false;
      break;
    }
  }

  if (accepted) {

    float position_offset_range(0.05f);
    int position_sampling_steps(2);
    float rotation_offset_range(1.f);
    int rotation_sampling_steps(2);

    double position_step_size(position_offset_range / double(position_sampling_steps - 1));
    if(position_step_size == 0.0) {
      position_step_size = 1.0;
    }

    double rotation_step_size(rotation_offset_range / double(rotation_sampling_steps - 1));
    if(rotation_step_size == 0.0) {
      rotation_step_size = 1.0;
    }

    auto screen_shot(retrieve_screen_shot(optimal_transform));
    auto optimal_error(error_function(current_photo_, screen_shot));

    double lowest_error(optimal_error);
    double highest_error(optimal_error);

    for (double x(-position_offset_range * 0.5); x <= position_offset_range * 0.5; x+=position_step_size) {
      for (double y(-position_offset_range * 0.5); y <= position_offset_range * 0.5; y+=position_step_size) {
        for (double z(-position_offset_range * 0.5); z <= position_offset_range * 0.5; z+=position_step_size) {
          for (double rot_x(-rotation_offset_range * 0.5); rot_x <= rotation_offset_range * 0.5; rot_x+=rotation_step_size) {
            auto current_x_rot(scm::math::make_rotation(rot_x, 1.0, 0.0, 0.0));
            for (double rot_y(-rotation_offset_range * 0.5); rot_y <= rotation_offset_range * 0.5; rot_y+=rotation_step_size) {
              auto current_y_rot(scm::math::make_rotation(rot_y, 0.0, 1.0, 0.0));
              for (double rot_z(-rotation_offset_range * 0.5); rot_z <= rotation_offset_range * 0.5; rot_z+=rotation_step_size) {
                auto current_z_rot(scm::math::make_rotation(rot_z, 0.0, 0.0, 1.0));

                auto current_difference(scm::math::make_translation(x, y, z) *
                                        current_y_rot * current_x_rot * current_z_rot);
                auto current_transform(optimal_transform * current_difference);

                screen_shot = retrieve_screen_shot(current_transform);
                auto current_error(error_function(current_photo_, screen_shot));
                if (current_error < lowest_error) {
                  lowest_error = current_error;
                  accepted = false;
                }

                if (current_error > highest_error) {
                  highest_error = current_error;
                }

              }
            }
          }
        }
      }
    }

    // check if found lowest error is actually significantly smaller than optimal error
    if (!accepted) {
      std::cout << "lowest_error " << lowest_error << std::endl;
      std::cout << "highest_error " << highest_error << std::endl;
      std::cout << "optimal_error " << optimal_error << std::endl;

      double error_ratio((optimal_error - lowest_error)/highest_error);
      std::cout << "error_ratio " << error_ratio << std::endl;

      if (error_ratio <= 0.1) {
        accepted = true;
      }
    }
  }


  return accepted;

}








