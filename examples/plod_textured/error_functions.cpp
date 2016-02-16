#include "error_functions.hpp"
#include "classification_functions.hpp"

#include <gua/utils.hpp>

float intensity_znssd(cv::Mat const& photo, cv::Mat const& screen_shot_in) {
  gua::Timer timer;
  timer.start();

  cv::Mat screen_shot;
  screen_shot_in.copyTo(screen_shot);

  cv::Mat mask;
  cv::threshold(screen_shot, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;
  photo.copyTo(masked_photo, mask);
  masked_photo.convertTo(masked_photo, CV_8UC1);
  cv::equalizeHist(masked_photo, masked_photo);
  cv::equalizeHist(screen_shot, screen_shot);

  screen_shot.convertTo(screen_shot, CV_32FC1, 1.0/255.0);
  masked_photo.convertTo(masked_photo, CV_32FC1, 1.0/255.0);

  // calculate zero-mean normalized sum of squared differences

  screen_shot = screen_shot - cv::mean(screen_shot);
  masked_photo = masked_photo - cv::mean(masked_photo);

  cv::Mat diff;
  cv::subtract(screen_shot, masked_photo, diff);
  cv::multiply(diff, diff, diff);

  double total_error = cv::sum(diff)[0] / (screen_shot.size().width * screen_shot.size().height);
  // std::cout << "Time: " << timer.get_elapsed() << std::endl;
  return total_error;
}

float intensity_znssd_clustered(cv::Mat const& photo, cv::Mat const& screen_shot_in) {
  gua::Timer timer;

  cv::Mat screen_shot;
  screen_shot_in.copyTo(screen_shot);

  cv::Mat mask;
  cv::threshold(screen_shot, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;
  photo.copyTo(masked_photo, mask);

  std::vector<float> photo_centers;
  cv::Mat photo_cluster_labels(compute_cluster_labels(masked_photo, 3, photo_centers));


  int highest_label(0);
  for (int i(0); i < photo_centers.size(); ++i) {
    if (photo_centers[i] > photo_centers[highest_label]) {
      highest_label = i;
    }
  }

  masked_photo.convertTo(masked_photo, CV_8UC1);
  cv::equalizeHist(masked_photo, masked_photo);
  cv::equalizeHist(screen_shot, screen_shot);

  screen_shot.convertTo(screen_shot, CV_32FC1, 1.0/255.0);
  masked_photo.convertTo(masked_photo, CV_32FC1, 1.0/255.0);

  // calculate zero-mean normalized sum of squared differences

  screen_shot = screen_shot - cv::mean(screen_shot);
  masked_photo = masked_photo - cv::mean(masked_photo);

  double total_error(1.0);
  int    considered_pixels(0);

  timer.start();
  for (int y(0); y < photo_cluster_labels.rows; ++y) {
    for (int x(0); x < photo_cluster_labels.cols; ++x) {
      int label(photo_cluster_labels.at<int>(y, x));
      if (label == highest_label) {
        float diff(screen_shot.at<float>(y,x) - masked_photo.at<float>(y,x));
        total_error += diff * diff;
        ++considered_pixels;
      }
    }
  }
  // std::cout << "Summation time: " << timer.get_elapsed() << std::endl;
  // screen_shot.convertTo(screen_shot, CV_8UC1, 255);
  // masked_photo.convertTo(masked_photo, CV_8UC1, 255);

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/masked_photo.png", masked_photo);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot.png", screen_shot);

  total_error /= 1.0 * considered_pixels;
  return total_error;
}

float blurred_gradient_znssd(cv::Mat const& photo, cv::Mat const& screen_shot_in) {
  gua::Timer timer;
  timer.start();

  cv::Mat mask;
  cv::threshold(screen_shot_in, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;

  photo.copyTo(masked_photo, mask);
  masked_photo.convertTo(masked_photo, CV_8UC1);

  cv::Size blur_kernel(5,5);

  cv::Mat screen_shot, photo_filtered;
  cv::adaptiveBilateralFilter(screen_shot_in, screen_shot, blur_kernel, 0.0);
  cv::adaptiveBilateralFilter(masked_photo, photo_filtered, blur_kernel, 0.0);

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot_filtered.png", screen_shot);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo_filtered.png", photo_filtered);

  cv::Mat dummy;
  double screen_shot_threshold(cv::threshold(screen_shot, dummy, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
  cv::Canny(screen_shot, screen_shot, screen_shot_threshold * 0.5, screen_shot_threshold, 3);

  double photo_threshold(cv::threshold(photo_filtered, dummy, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
  cv::Canny(photo_filtered, photo_filtered, photo_threshold * 0.5, photo_threshold, 3);

  // shrink mask to remove unwanted edges
  int erosion_size = 2;
  cv::Mat element(cv::getStructuringElement(
                  cv::MORPH_CROSS,
                  cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                  cv::Point(erosion_size, erosion_size)));

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/mask_before.png", mask);

  cv::erode(mask, mask, element);
  cv::Mat screen_shot_gradient;
  cv::Mat photo_gradient;
  screen_shot.copyTo(screen_shot_gradient, mask);
  photo_filtered.copyTo(photo_gradient, mask);

  // TODO: What tolerance of error (degree/m) does the chosen blur kernel resamble?
  // 1° of rotation ~ 14 - 15 px
  cv::Size error_kernel(11,11);
  cv::GaussianBlur(screen_shot_gradient, screen_shot_gradient, error_kernel, 0.0, 0.0);
  cv::GaussianBlur(photo_gradient, photo_gradient, error_kernel, 0.0, 0.0);

  cv::equalizeHist(screen_shot_gradient, screen_shot_gradient);
  cv::equalizeHist(photo_gradient, photo_gradient);

  // int voting_threshold(80);
  // int min_line_length(20);
  // int max_line_gap(10);

  // cv::Mat screen_shot_lines(screen_shot.size().height, screen_shot.size().width, CV_8UC1);
  // std::vector<cv::Vec4i> lines;
  // cv::HoughLinesP(screen_shot, lines, 1, CV_PI/180.0, voting_threshold, min_line_length, max_line_gap);
  // for (int i(0); i < lines.size(); ++i) {
  //   cv::line(screen_shot_lines,
  //            cv::Point(lines[i][0], lines[i][1]),
  //            cv::Point(lines[i][2], lines[i][3]),
  //            cv::Scalar(255,255,255), 1);
  // }

  // cv::Mat photo_lines(screen_shot.size().height, screen_shot.size().width, CV_8UC1);
  // lines = std::vector<cv::Vec4i>();
  // cv::HoughLinesP(masked_photo, lines, 1, CV_PI/180.0, voting_threshold, min_line_length, max_line_gap);
  // for (int i(0); i < lines.size(); ++i) {
  //   cv::line(photo_lines,
  //            cv::Point(lines[i][0], lines[i][1]),
  //            cv::Point(lines[i][2], lines[i][3]),
  //            cv::Scalar(255,255,255), 1);
  // }


  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/mask.png", mask);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot_gradient.png", screen_shot_gradient);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot_lines.png", screen_shot_lines);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo_gradient.png", photo_gradient);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo_lines.png", photo_lines);

  screen_shot_gradient.convertTo(screen_shot_gradient, CV_32FC1, 1.0/255.0);
  photo_gradient.convertTo(photo_gradient, CV_32FC1, 1.0/255.0);

  // calculate zero-mean normalized sum of squared differences

  screen_shot_gradient = screen_shot_gradient - cv::mean(screen_shot_gradient);
  photo_gradient = photo_gradient - cv::mean(photo_gradient);

  cv::Mat diff;

  cv::subtract(screen_shot_gradient, photo_gradient, diff);
  cv::multiply(diff, diff, diff);

  double total_error = cv::sum(diff)[0] / (screen_shot_gradient.size().width * screen_shot_gradient.size().height);
  // std::cout << "Time: " << timer.get_elapsed() << std::endl;
  return total_error;
  // return 0.0;
}
