#include "classification_functions.hpp"
#include "ImageClusterGenerator.hpp"

bool intensity_cluster_ratio(cv::Mat const& photo, cv::Mat const& screen_shot_in) {

  return ImageClusterGenerator::instance()->calculate_center_ratio() <= 0.5f;
}
