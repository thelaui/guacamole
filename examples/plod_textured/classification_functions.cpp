#include "classification_functions.hpp"

#include <gua/utils.hpp>

bool intensity_cluster_ratio(cv::Mat const& photo, cv::Mat const& screen_shot_in) {

  cv::Mat screen_shot;
  screen_shot_in.copyTo(screen_shot);

  cv::Mat mask;
  cv::threshold(screen_shot, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;
  photo.copyTo(masked_photo, mask);

  std::vector<float> photo_centers;
  cv::Mat photo_cluster_labels(compute_cluster_labels(masked_photo, 3, photo_centers));

  std::sort(photo_centers.begin(), photo_centers.end());

  float highest(photo_centers[photo_centers.size()-1]);
  float mid(photo_centers[photo_centers.size()-2]);
  float lowest(photo_centers[0]);
  float ratio((mid - lowest) / (highest - lowest));
  std::cout << "highest " << highest << std::endl;
  std::cout << "mid " << mid << std::endl;
  std::cout << "lowest " << lowest << std::endl;
  std::cout << "ratio " << ratio << std::endl;

  return ratio <= 0.5f;
}

cv::Mat const compute_cluster_labels(cv::Mat const& image_in, int cluster_count, std::vector<float>& centers) {
  cv::Mat cluster_labels;
  cv::TermCriteria criteria;
  criteria.epsilon = 1.0;
  criteria.maxCount = 10;
  criteria.type = cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS;

  int trials(10);

  cv::Mat image;
  image_in.copyTo(image);
  image.convertTo(image, CV_32FC1);
  int image_rows(image.rows);
  int image_cols(image.cols);

  image = image.reshape(0, image_rows * image_cols);

  cv::Mat cv_centers;
  gua::Timer timer;
  timer.start();
  double cluster_compactness(cv::kmeans(image, cluster_count, cluster_labels, criteria, trials, cv::KMEANS_RANDOM_CENTERS, cv_centers));
  // std::cout << "Clustering time: " << timer.get_elapsed() << std::endl;
  cluster_labels = cluster_labels.reshape(0, image_rows);

  for (int i(0); i < cv_centers.rows; ++i) {
    centers.push_back(cv_centers.at<float>(i));
  }

  return cluster_labels;
}
