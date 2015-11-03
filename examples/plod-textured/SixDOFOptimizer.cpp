#include "SixDOFOptimizer.hpp"

#include <iostream>

#include <scm/gl_core/math.h>

#include <unistd.h>

scm::math::mat4f const SixDOFOptimizer::run() {

  float position_step_size(position_offset_range / float(position_sampling_steps - 1));
  float rotation_step_size(rotation_offset_range / float(rotation_sampling_steps - 1));
  auto best_transform(initial_transform);
  float lowest_error(std::numeric_limits<float>::max());

  for (float x(-position_offset_range * 0.5); x <= position_offset_range * 0.5; x+=position_step_size) {
    for (float y(-position_offset_range * 0.5); y <= position_offset_range * 0.5; y+=position_step_size) {
      for (float z(-position_offset_range * 0.5); z <= position_offset_range * 0.5; z+=position_step_size) {
        for (float rot_x(-rotation_offset_range * 0.5); rot_x <= rotation_offset_range * 0.5; rot_x+=rotation_step_size) {
          auto current_x_rot(scm::math::make_rotation(rot_x, 1.f, 0.f, 0.f));
          for (float rot_y(-rotation_offset_range * 0.5); rot_y <= rotation_offset_range * 0.5; rot_y+=rotation_step_size) {
            auto current_y_rot(scm::math::make_rotation(rot_y, 0.f, 1.f, 0.f));
            for (float rot_z(-rotation_offset_range * 0.5); rot_z <= rotation_offset_range * 0.5; rot_z+=rotation_step_size) {
              auto current_z_rot(scm::math::make_rotation(rot_z, 0.f, 0.f, 1.f));

              auto current_transform(initial_transform * scm::math::make_translation(x, y, z) *
                                     current_y_rot * current_x_rot * current_z_rot);
              auto current_error(error_function(current_transform));
              if (current_error < lowest_error) {
                lowest_error = current_error;
                best_transform = current_transform;
              }

            }
          }
        }
      }
    }
  }

  return best_transform;
}
