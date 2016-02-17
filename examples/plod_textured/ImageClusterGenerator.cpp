#include "ImageClusterGenerator.hpp"

#include <gua/utils.hpp>

ImageClusterGenerator::ImageClusterGenerator()
  : centers_()
  , cluster_labels_()
{}

void ImageClusterGenerator::init(cv::Mat const& photo,
                                 cv::Mat const& screen_shot_in,
                                 int cluster_count) {

  centers_.clear();
  cluster_labels_ = cv::Mat();

  cv::Mat screen_shot;
  screen_shot_in.copyTo(screen_shot);

  cv::Mat mask;
  cv::threshold(screen_shot, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;
  photo.copyTo(masked_photo, mask);

  cv::TermCriteria criteria;
  criteria.epsilon = 1.0;
  criteria.maxCount = 10;
  criteria.type = cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS;

  int trials(10);

  masked_photo.convertTo(masked_photo, CV_32FC1);
  int image_rows(masked_photo.rows);
  int image_cols(masked_photo.cols);

  masked_photo = masked_photo.reshape(0, image_rows * image_cols);

  cv::Mat cv_centers;
  gua::Timer timer;
  timer.start();

  double cluster_compactness(cv::kmeans(masked_photo, cluster_count, cluster_labels_, criteria, trials, cv::KMEANS_RANDOM_CENTERS, cv_centers));
  // std::cout << "Clustering time: " << timer.get_elapsed() << std::endl;
  cluster_labels_ = cluster_labels_.reshape(0, image_rows);

  for (int i(0); i < cv_centers.rows; ++i) {
    centers_.push_back(cv_centers.at<float>(i));
  }

}

float ImageClusterGenerator::calculate_center_ratio() const {

  std::vector<float> sorted_centers(centers_);

  std::sort(sorted_centers.begin(), sorted_centers.end());

  float highest(sorted_centers[sorted_centers.size()-1]);
  float mid(sorted_centers[sorted_centers.size()-2]);
  float lowest(sorted_centers[0]);
  float ratio((mid - lowest) / (highest - lowest));
  std::cout << "highest " << highest << std::endl;
  std::cout << "mid " << mid << std::endl;
  std::cout << "lowest " << lowest << std::endl;
  std::cout << "ratio " << ratio << std::endl;

  return ratio;
}
