#include "classification_functions.hpp"

#include <gua/utils.hpp>

bool intensity_threshold(cv::Mat const& photo, cv::Mat const& screen_shot) {
  gua::Timer timer;
  timer.start();

  cv::Mat mask;
  cv::threshold(screen_shot, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;
  photo.copyTo(masked_photo, mask);
  masked_photo.convertTo(masked_photo, CV_32FC1);

  int histogram_size(256);
  float range[] = { 0, 256 } ;
  const float* histogram_range = { range };

  cv::Mat histogram;
  cv::calcHist(&masked_photo, 1, 0, cv::Mat(), histogram, 1, &histogram_size, &histogram_range, true, false);

  int hist_w = 1280; int hist_h = 800;
  int bin_w = cvRound( (double) hist_w/histogram_size );
  cv::Mat histogram_image(hist_h, hist_w, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::normalize(histogram, histogram, 0, histogram_image.rows, cv::NORM_MINMAX, -1, cv::Mat());

  // clustering
  cv::Mat cluster_labels;
  cv::TermCriteria criteria;
  criteria.epsilon = 1.0;
  criteria.maxCount = 10;
  criteria.type = cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS;

  int cluster_count(3);
  int trials(10);

  masked_photo = masked_photo.reshape(0, masked_photo.rows * masked_photo.cols);
  cv::Mat centers;
  double cluster_compactness(cv::kmeans(masked_photo, cluster_count, cluster_labels, criteria, trials, cv::KMEANS_RANDOM_CENTERS, centers));

  cluster_labels = cluster_labels.reshape(0, screen_shot.rows);

  float color_scale(255.f / cluster_count);
  cv::Mat color_lookup(cluster_count, 1, CV_8UC1, cv::Scalar(0,0,0));

  for (int i(1); i <= cluster_count; ++i) {
    int intensity(int(color_scale * i));
    color_lookup.at<int>(i-1, 0, 0) = intensity;
  }

  cv::cvtColor(color_lookup, color_lookup, CV_GRAY2RGB);
  cv::applyColorMap(color_lookup, color_lookup, cv::COLORMAP_JET);

  cv::Mat clustered_image(screen_shot.rows, screen_shot.cols, CV_8UC3);

  for (int y(0); y < screen_shot.rows; ++y) {
    for (int x(0); x < screen_shot.cols; ++x) {
      int label(cluster_labels.at<int>(y, x, 0));
      cv::Vec3b color(color_lookup.at<cv::Vec3b>(label, 0, 0));
      clustered_image.at<cv::Vec3b>(y, x, 0) = color;
    }
  }

  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/clustered_image.png", clustered_image);

  // for (int i(1); i < histogram_size; ++i) {

  //     int label(cluster_labels.at<int>(i));
  //     cv::Scalar line_color(color_lookup.at<cv::Vec3b>(label, 0, 0));

  //     cv::line(histogram_image, cv::Point(bin_w*(i-1), hist_h - cvRound(histogram.at<float>(i-1))) ,
  //              cv::Point(bin_w*(i), hist_h - cvRound(histogram.at<float>(i))),
  //              line_color, 2, 8, 0);

  //     // std::cout << cluster_labels.at<int>(i) << std::endl;
  // }

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/histogram_image.png", histogram_image);


  // std::cout << "Time: " << timer.get_elapsed() << std::endl;
  // return cv::sum(classification_image)[0] > (screen_shot.size().width * screen_shot.size().height) / 2;
  return false;
}
