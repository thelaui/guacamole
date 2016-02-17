#include "ErrorFunctionSampler.hpp"
#include "ImageClusterGenerator.hpp"

#include <iostream>

#include <scm/gl_core/math.h>
#include <gua/math.hpp>

#include <unistd.h>
#include <fstream>

void ErrorFunctionSampler::sample_dimension(int dimension, double min, double max, double step_size) {

  current_photo_ = retrieve_photo();

  cv::Mat screen_shot(retrieve_screen_shot(initial_transform));

  ImageClusterGenerator::instance()->init(current_photo_, screen_shot, 3);

  for (double step_value(min); step_value <= max; step_value += step_size) {

    scm::math::mat4d current_translation(scm::math::mat4d::identity());
    scm::math::mat4d current_rotation(scm::math::mat4d::identity());

    if (dimension < 3) {
      scm::math::vec3d translation_vector(0.0);
      translation_vector[dimension] = step_value;
      current_translation = scm::math::make_translation(translation_vector);
    } else if (dimension < 6) {
      scm::math::vec3d rotation_vector(0.0);
      rotation_vector[dimension - 3] = 1.0;
      current_translation = scm::math::make_rotation(step_value, rotation_vector);
    }

    auto current_difference(current_translation * current_rotation);
    auto current_transform(initial_transform * current_difference);

    screen_shot = retrieve_screen_shot(current_transform);
    auto current_error(error_function(current_photo_, screen_shot));
    std::cout << step_value << " " << current_error << std::endl;
    // auto translation(gua::math::get_translation(current_transform));
    // std::cout << translation << std::endl;

  }

}
