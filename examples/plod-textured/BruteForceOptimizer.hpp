#ifndef BRUTE_FORCE_OTIMIZER_HPP
#define BRUTE_FORCE_OTIMIZER_HPP

#include <functional>

#include <scm/core/math.h>

#include "opencv2/opencv.hpp"

class BruteForceOptimizer {

  public:

    float position_offset_range = 1.0; // in world coordinates
    int   position_sampling_steps = 2;

    float rotation_offset_range = 1.0; // in degrees
    int   rotation_sampling_steps = 2;

    scm::math::mat4f initial_transform = scm::math::mat4f::identity();

    std::function<cv::Mat()> retrieve_photo =
                                [](){return cv::Mat();};

    std::function<cv::Mat(scm::math::mat4f const&)> retrieve_screen_shot =
                                [](scm::math::mat4f const&){return cv::Mat();};

    // the error function has to compute an error based on two images
    // the cv::Mat passed as first argument is a photographic grayscale image,
    // the second is a grayscale screen shot
    std::function<float(cv::Mat const&, cv::Mat const&)> error_function =
                                [](cv::Mat const&, cv::Mat const&){return 0.0;};

    void run(scm::math::mat4f& optimal_transform,
             scm::math::mat4f& optimal_difference);

  private:

    cv::Mat current_photo_;

};

#endif  // BRUTE_FORCE_OTIMIZER_HPP
