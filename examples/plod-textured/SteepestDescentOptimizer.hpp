#ifndef STEEPEST_DESCENT_OTIMIZER_HPP
#define STEEPEST_DESCENT_OTIMIZER_HPP

#include <functional>

#include <scm/core/math.h>

#include "opencv2/opencv.hpp"

class SteepestDescentOptimizer {

  public:

    scm::math::mat4d initial_transform = scm::math::mat4d::identity();

    std::function<cv::Mat()> retrieve_photo =
                                [](){return cv::Mat();};

    std::function<cv::Mat(scm::math::mat4d const&)> retrieve_screen_shot =
                                [](scm::math::mat4d const&){return cv::Mat();};

    // the error function has to compute an error based on two images
    // the cv::Mat passed as first argument is a photographic grayscale image,
    // the second is a grayscale screen shot
    std::function<double(cv::Mat const&, cv::Mat const&)> error_function =
                                [](cv::Mat const&, cv::Mat const&){return 0.0;};


    void run(scm::math::mat4d& optimal_transform,
             scm::math::mat4d& optimal_difference);

    void run_round_robin(scm::math::mat4d& optimal_transform,
                         scm::math::mat4d& optimal_difference);

  private:

    scm::math::mat<double, 6, 1> get_gradient(
                               scm::math::mat4d const& central_transform) const;

    double get_gradient_for_dimension(scm::math::mat4d const& central_transform,
                                      int dimension) const;

    void update_step_length(scm::math::mat4d const& central_transform,
                            scm::math::mat<double, 6, 1> const& gradient);

    double current_step_length_ = 1.f;

    cv::Mat current_photo_;

};

#endif  // STEEPEST_DESCENT_OTIMIZER_HPP
