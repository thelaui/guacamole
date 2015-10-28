#ifndef SIX_DOF_OTIMIZER_HPP
#define SIX_DOF_OTIMIZER_HPP

#include <functional>

#include <scm/core/math.h>

class SixDOFOptimizer {

  public:

    float position_offset_range = 1.0; // in world coordinates
    int   position_sampling_steps = 2;

    float rotation_offset_range = 1.0; // in degrees
    int   rotation_sampling_steps = 2;

    scm::math::mat4f initial_transform = scm::math::mat4f::identity();

    // the error function has to compute an error based on a sampled
    // transformation
    std::function<float(scm::math::mat4f const&)> error_function =
                                        [](scm::math::mat4f const&){return 0.0;};

    scm::math::mat4f const run();

};

#endif  // SIX_DOF_OTIMIZER_HPP
