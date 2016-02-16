#include "classification_functions.hpp"

#include <gua/utils.hpp>

bool intensity_threshold(cv::Mat const& photo, cv::Mat const& screen_shot_in) {

  auto calc_cluster = [](cv::Mat& image, cv::Mat& result, int cluster_count){

    cv::Mat cluster_labels;
    cv::TermCriteria criteria;
    criteria.epsilon = 1.0;
    criteria.maxCount = 10;
    criteria.type = cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS;

    int trials(10);
    int image_rows(image.rows);
    int image_cols(image.cols);

    image = image.reshape(0, image_rows * image_cols);

    cv::Mat centers;
    double cluster_compactness(cv::kmeans(image, cluster_count, cluster_labels, criteria, trials, cv::KMEANS_RANDOM_CENTERS, centers));
    std::cout << "cluster_compactness " << cluster_compactness << std::endl;

    std::vector<float> sorted_centers;
    for (int i(0); i < centers.rows; ++i) {
      sorted_centers.push_back(centers.at<float>(i));
    }

    std::sort(sorted_centers.begin(), sorted_centers.end());

    float highest(sorted_centers[sorted_centers.size()-1]);
    float mid(sorted_centers[sorted_centers.size()-2]);
    float lowest(sorted_centers[0]);
    float ratio((mid - lowest) / (highest - lowest));
    std::cout << "highest " << highest << std::endl;
    std::cout << "mid " << mid << std::endl;
    std::cout << "lowest " << lowest << std::endl;
    std::cout << "ratio " << ratio << std::endl;

    cluster_labels = cluster_labels.reshape(0, image_rows);

    float color_scale(255.f / cluster_count);
    cv::Mat color_lookup(cluster_count, 1, CV_8UC1, cv::Scalar(0,0,0));

    for (int i(1); i <= cluster_count; ++i) {
      int intensity(int(color_scale * i));
      color_lookup.at<int>(i-1, 0, 0) = intensity;
    }

    cv::cvtColor(color_lookup, color_lookup, CV_GRAY2RGB);
    cv::applyColorMap(color_lookup, color_lookup, cv::COLORMAP_JET);

    cv::Mat clustered_image(image_rows, image_cols, CV_8UC3);

    for (int y(0); y < image_rows; ++y) {
      for (int x(0); x < image_cols; ++x) {
        int label(cluster_labels.at<int>(y, x, 0));
        cv::Vec3b color(color_lookup.at<cv::Vec3b>(label, 0, 0));
        clustered_image.at<cv::Vec3b>(y, x, 0) = color;
      }
    }

    result = clustered_image;
    return ratio;
  };

  cv::Mat screen_shot;
  screen_shot_in.copyTo(screen_shot);

  cv::Mat mask;
  cv::threshold(screen_shot, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;
  photo.copyTo(masked_photo, mask);
  masked_photo.convertTo(masked_photo, CV_32FC1);
  screen_shot.convertTo(screen_shot, CV_32FC1);

  std::cout << "photo clustering" << std::endl;
  cv::Mat photo_cluster;
  float photo_ratio(calc_cluster(masked_photo, photo_cluster, 4));

  std::cout << "screen shot clustering" << std::endl;
  cv::Mat screen_shot_cluster;
  float screen_shot_ratio(calc_cluster(screen_shot, screen_shot_cluster, 4));

  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo_cluster.png", photo_cluster);
  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot_cluster.png", screen_shot_cluster);

  // int histogram_size(256);
  // float range[] = { 0, 256 } ;
  // const float* histogram_range = { range };

  // cv::Mat histogram;
  // cv::calcHist(&masked_photo, 1, 0, cv::Mat(), histogram, 1, &histogram_size, &histogram_range, true, false);

  // int hist_w = 1280; int hist_h = 800;
  // int bin_w = cvRound( (double) hist_w/histogram_size );
  // cv::Mat histogram_image(hist_h, hist_w, CV_8UC3, cv::Scalar(255, 255, 255));
  // cv::normalize(histogram, histogram, 0, histogram_image.rows, cv::NORM_MINMAX, -1, cv::Mat());

  // for (int i(1); i < histogram_size; ++i) {

  //     int label(cluster_labels.at<int>(i));
  //     cv::Scalar line_color(color_lookup.at<cv::Vec3b>(label, 0, 0));

  //     cv::line(histogram_image, cv::Point(bin_w*(i-1), hist_h - cvRound(histogram.at<float>(i-1))) ,
  //              cv::Point(bin_w*(i), hist_h - cvRound(histogram.at<float>(i))),
  //              line_color, 2, 8, 0);

  //     // std::cout << cluster_labels.at<int>(i) << std::endl;
  // }

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/histogram_image.png", histogram_image);


  // return cv::sum(classification_image)[0] > (screen_shot.size().width * screen_shot.size().height) / 2;
  return photo_ratio <= 0.5f;
}
