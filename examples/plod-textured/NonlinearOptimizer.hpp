#ifndef NONLINEAR_OTIMIZER_HPP
#define NONLINEAR_OTIMIZER_HPP

#include <functional>

#include <scm/core/math.h>

class NonlinearOptimizer {

  public:

    scm::math::mat4f initial_transform = scm::math::mat4f::identity();

    // the error function has to compute an error based on a sampled
    // transformation
    std::function<float(scm::math::mat4f const&)> error_function =
                                        [](scm::math::mat4f const&){return 0.0;};

    void run(scm::math::mat4f& optimal_transform,
             scm::math::mat4f& optimal_difference);

  private:

    std::vector<float> get_descent_direction(scm::math::mat4f const& central_transform) const;
    // float get_step_length()

};

#endif  // NONLINEAR_OTIMIZER_HPP
