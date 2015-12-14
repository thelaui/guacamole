#ifndef ERROR_FUNCTION_SAMPLER_HPP
#define ERROR_FUNCTION_SAMPLER_HPP

#include <functional>

#include <scm/core/math.h>

#include "opencv2/opencv.hpp"

class ErrorFunctionSampler {

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


    void sample_dimension(int dimension, double min, double max, double step_size);


  private:

    cv::Mat current_photo_;

};

#endif  // ERROR_FUNCTION_SAMPLER_HPP
