#ifndef CLASSIFICATION_FUNCTIONS_HPP
#define CLASSIFICATION_FUNCTIONS_HPP

#include <gua/math.hpp>

#include "opencv2/opencv.hpp"

bool intensity_threshold(cv::Mat const& photo, cv::Mat const& screen_shot);

#endif  // CLASSIFICATION_FUNCTIONS_HPP
