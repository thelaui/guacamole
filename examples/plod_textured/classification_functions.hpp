#ifndef CLASSIFICATION_FUNCTIONS_HPP
#define CLASSIFICATION_FUNCTIONS_HPP

#include <gua/math.hpp>
#include <gua/utils/Singleton.hpp>

#include "opencv2/opencv.hpp"

bool intensity_cluster_ratio(cv::Mat const& photo, cv::Mat const& screen_shot);

#endif  // CLASSIFICATION_FUNCTIONS_HPP
