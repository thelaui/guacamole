#ifndef ERROR_FUNCTIONS_HPP
#define ERROR_FUNCTIONS_HPP

#include <gua/math.hpp>

#include "opencv2/opencv.hpp"

// zero mean normalized sum of squared differences
float intensity_znssd(cv::Mat const& photo, cv::Mat const& screen_shot);

//
float summed_distances_to_closest_line(cv::Mat const& photo, cv::Mat const& screen_shot);

#endif  // ERROR_FUNCTIONS_HPP
