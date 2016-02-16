#ifndef CLASSIFICATION_FUNCTIONS_HPP
#define CLASSIFICATION_FUNCTIONS_HPP

#include <gua/math.hpp>

#include "opencv2/opencv.hpp"

cv::Mat const compute_cluster_labels(cv::Mat const& image, int cluster_count, std::vector<float>& centers);

bool intensity_cluster_ratio(cv::Mat const& photo, cv::Mat const& screen_shot);

#endif  // CLASSIFICATION_FUNCTIONS_HPP
