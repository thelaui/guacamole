#include "error_functions.hpp"

#include <gua/utils.hpp>

float intensity_znssd(cv::Mat const& photo, cv::Mat const& screen_shot_in) {
  gua::Timer timer;
  timer.start();


  cv::Mat mask;
  cv::threshold(screen_shot_in, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;

  photo.copyTo(masked_photo, mask);
  masked_photo.convertTo(masked_photo, CV_8UC1);
  cv::equalizeHist(masked_photo, masked_photo);

  cv::Size blur_kernel(10,10);

  cv::Mat screen_shot;
  cv::blur(screen_shot_in, screen_shot, blur_kernel);
  cv::blur(masked_photo, masked_photo, blur_kernel);

  // cv::Mat screen_shot_grad_x;
  // cv::Mat screen_shot_grad_y;
  // cv::Sobel(screen_shot, screen_shot_grad_x, CV_8UC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  // cv::convertScaleAbs(screen_shot_grad_x, screen_shot_grad_x);

  // cv::Sobel(screen_shot, screen_shot_grad_y, CV_8UC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
  // cv::convertScaleAbs(screen_shot_grad_y, screen_shot_grad_y);

  // cv::addWeighted(screen_shot_grad_x, 0.5, screen_shot_grad_y, 0.5, 0, screen_shot);

  // int morph_size = static_cast<int>(current_lens_radius) * 2;
  // int morph_size = 2;
  // cv::Mat element(cv::getStructuringElement(
  //                 cv::MORPH_RECT,
  //                 cv::Size( 2*morph_size + 1, 2*morph_size+1 ),
  //                 cv::Point( morph_size, morph_size )));


  // int iterations(1);

  // cv::morphologyEx(screen_shot, screen_shot, cv::MORPH_OPEN, element, cv::Point(-1,-1), iterations);
  // cv::morphologyEx(screen_shot, screen_shot, cv::MORPH_CLOSE, element, cv::Point(-1,-1), iterations);

  // cv::Mat masked_photo_grad_x;
  // cv::Mat masked_photo_grad_y;
  // cv::Sobel(masked_photo, masked_photo_grad_x, CV_8UC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  // cv::convertScaleAbs(masked_photo_grad_x, masked_photo_grad_x);

  // cv::Sobel(masked_photo, masked_photo_grad_y, CV_8UC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
  // cv::convertScaleAbs(masked_photo_grad_y, masked_photo_grad_y);

  // cv::addWeighted(masked_photo_grad_x, 0.5, masked_photo_grad_y, 0.5, 0, masked_photo);

  // cv::morphologyEx(masked_photo, masked_photo, cv::MORPH_OPEN, element, cv::Point(-1,-1), iterations);
  // cv::morphologyEx(masked_photo, masked_photo, cv::MORPH_CLOSE, element, cv::Point(-1,-1), iterations);

  // cv::blur(screen_shot, screen_shot, blur_kernel * 2);
  // cv::blur(masked_photo, masked_photo, blur_kernel * 2);

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/" + std::to_string(function_calls) + "_screen_shot.png", screen_shot);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/" + std::to_string(function_calls) + "_photo.png", masked_photo);
  // ++function_calls;

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

float summed_distances_to_closest_line(cv::Mat const& photo, cv::Mat const& screen_shot_in) {
  gua::Timer timer;
  timer.start();

  cv::Mat mask;
  cv::threshold(screen_shot_in, mask, 0, 255, cv::THRESH_BINARY);

  cv::Mat masked_photo;

  photo.copyTo(masked_photo, mask);
  masked_photo.convertTo(masked_photo, CV_8UC1);
  // cv::equalizeHist(masked_photo, masked_photo);

  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot_eq.png", screen_shot_in);
  // cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo_eq.png", masked_photo);

  cv::Size blur_kernel(5,5);

  cv::Mat screen_shot;
  cv::blur(screen_shot_in, screen_shot, blur_kernel);
  cv::blur(masked_photo, masked_photo, blur_kernel);

  cv::Mat dummy;
  double screen_shot_threshold(cv::threshold(screen_shot, dummy, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
  cv::Canny(screen_shot, screen_shot, screen_shot_threshold * 0.5, screen_shot_threshold, 3);

  double photo_threshold(cv::threshold(masked_photo, dummy, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU));
  cv::Canny(masked_photo, masked_photo, photo_threshold * 0.5, photo_threshold, 3);

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
  masked_photo.copyTo(photo_gradient, mask);

  // TODO: What tolerance of error (degree/m) does the chosen blur kernel resamble?
  // cv::Size error_kernel(20,20);
  // cv::blur(screen_shot_gradient, screen_shot_gradient, error_kernel);
  // cv::blur(photo_gradient, photo_gradient, error_kernel);

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
