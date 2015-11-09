#ifndef BRUTE_FORCE_OTIMIZER_HPP
#define BRUTE_FORCE_OTIMIZER_HPP

#include <functional>

#include <scm/core/math.h>

class BruteForceOptimizer {

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

    void run(scm::math::mat4f& optimal_transform,
             scm::math::mat4f& optimal_difference);

};

#endif  // BRUTE_FORCE_OTIMIZER_HPP
