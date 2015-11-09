#include "SixDOFOptimizer.hpp"

#include <iostream>

#include <scm/gl_core/math.h>

#include <unistd.h>
#include <fstream>

void SixDOFOptimizer::run(scm::math::mat4f& optimal_transform, scm::math::mat4f& optimal_difference) {

  float position_step_size(position_offset_range / float(position_sampling_steps - 1));
  if(position_step_size == 0.f) {
    position_step_size = 1.f;
  }

  float rotation_step_size(rotation_offset_range / float(rotation_sampling_steps - 1));
  if(rotation_step_size == 0.f) {
    rotation_step_size = 1.f;
  }

  optimal_transform = initial_transform;
  float lowest_error(std::numeric_limits<float>::max());
  int output_count(0);

  for (float x(-position_offset_range * 0.5); x <= position_offset_range * 0.5; x+=position_step_size) {
    for (float y(-position_offset_range * 0.5); y <= position_offset_range * 0.5; y+=position_step_size) {
      for (float z(-position_offset_range * 0.5); z <= position_offset_range * 0.5; z+=position_step_size) {
        for (float rot_x(-rotation_offset_range * 0.5); rot_x <= rotation_offset_range * 0.5; rot_x+=rotation_step_size) {
          // std::fstream ofstr("/home/tosa2305/Desktop/thesis/data/untracked/error_data/many_features/x_" + std::to_string(output_count++) + "_" + std::to_string(rot_x) + ".tsv", std::ios::out);
          // std::fstream ofstr("/home/tosa2305/Desktop/thesis/data/untracked/error_data/few_features/x_" + std::to_string(output_count++) + "_" + std::to_string(rot_x) + ".tsv", std::ios::out);
          auto current_x_rot(scm::math::make_rotation(rot_x, 1.f, 0.f, 0.f));
          for (float rot_y(-rotation_offset_range * 0.5); rot_y <= rotation_offset_range * 0.5; rot_y+=rotation_step_size) {
            auto current_y_rot(scm::math::make_rotation(rot_y, 0.f, 1.f, 0.f));
            for (float rot_z(-rotation_offset_range * 0.5); rot_z <= rotation_offset_range * 0.5; rot_z+=rotation_step_size) {
              auto current_z_rot(scm::math::make_rotation(rot_z, 0.f, 0.f, 1.f));

              auto current_difference(scm::math::make_translation(x, y, z) *
                                      current_y_rot * current_x_rot * current_z_rot);
              auto current_transform(initial_transform * current_difference);
              // auto current_transform(initial_transform *
              //                        current_y_rot * current_x_rot * current_z_rot);
              auto current_error(error_function(current_transform));
              // ofstr << rot_y << " " << rot_z << " " << current_error << std::endl;
              if (current_error < lowest_error) {
                lowest_error = current_error;
                optimal_transform = current_transform;
                optimal_difference = current_difference;
              }

            }
          }
          // ofstr.close();
        }
      }
    }
  }

}
