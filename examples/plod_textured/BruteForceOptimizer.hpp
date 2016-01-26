#ifndef BRUTE_FORCE_OTIMIZER_HPP
#define BRUTE_FORCE_OTIMIZER_HPP

#include <functional>

#include <scm/core/math.h>

#include "opencv2/opencv.hpp"

class BruteForceOptimizer {

  public:

    double position_offset_range = 1.0; // in world coordinates
    int   position_sampling_steps = 2;

    double rotation_offset_range = 1.0; // in degrees
    int   rotation_sampling_steps = 2;

    scm::math::mat4d initial_transform = scm::math::mat4d::identity();

    std::function<cv::Mat()> retrieve_photo =
                                [](){return cv::Mat();};

    std::function<cv::Mat(scm::math::mat4d const&)> retrieve_screen_shot =
                                [](scm::math::mat4d const&){return cv::Mat();};

    bool use_cv_error_function = false;

    // the error function has to compute an error based on two images
    // the cv::Mat passed as first argument is a photographic grayscale image,
    // the second is a grayscale screen shot
    std::function<double(cv::Mat const&, cv::Mat const&)> cv_error_function =
                                [](cv::Mat const&, cv::Mat const&){return 0.0;};

    std::function<double(scm::math::mat4d const&)> generic_error_function =
                                [](scm::math::mat4d const&){return 0.0;};

    void run(scm::math::mat4d& optimal_transform,
             scm::math::mat4d& optimal_difference);

  private:

    cv::Mat current_photo_;

};

#endif  // BRUTE_FORCE_OTIMIZER_HPP
