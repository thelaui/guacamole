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

  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot_eq.png", screen_shot_in);
  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo_eq.png", masked_photo);

  cv::Size blur_kernel(1,1);

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


  // cv::Mat masked_photo_grad_x;
  // cv::Mat masked_photo_grad_y;
  // cv::Sobel(masked_photo, masked_photo_grad_x, CV_8UC1, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  // cv::convertScaleAbs(masked_photo_grad_x, masked_photo_grad_x);

  // cv::Sobel(masked_photo, masked_photo_grad_y, CV_8UC1, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
  // cv::convertScaleAbs(masked_photo_grad_y, masked_photo_grad_y);

  // cv::addWeighted(masked_photo_grad_x, 0.5, masked_photo_grad_y, 0.5, 0, masked_photo);

  // cv::Canny(screen_shot, screen_shot, 200, 600, 3);
  // cv::Canny(masked_photo, masked_photo, 200, 600, 3);


  cv::Laplacian(screen_shot, screen_shot, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
  cv::convertScaleAbs(screen_shot, screen_shot);

  cv::Laplacian(masked_photo, masked_photo, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
  cv::convertScaleAbs(masked_photo, masked_photo);

  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/screen_shot.png", screen_shot);
  cv::imwrite("/home/tosa2305/Desktop/plod-textured-out/photo.png", masked_photo);

  return 0.0;
}
