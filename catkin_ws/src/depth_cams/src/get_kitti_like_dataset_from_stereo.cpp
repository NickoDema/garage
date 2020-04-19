#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;
using namespace std::chrono;

std::vector<cv::Point3d> Generate3DPoints();

int main(int argc, char *argv[]){

  string dataset_dir_path;
  if (argc > 1) dataset_dir_path = argv[1];

  string right_cam_data_path = dataset_dir_path + "/image_02/data/";
  string left_cam_data_path  = dataset_dir_path + "/image_03/data/";

  string right_cam_timestamps_file = dataset_dir_path + "/image_02/timestamps.txt";
  string left_cam_timestamps_file  = dataset_dir_path + "/image_03/timestamps.txt";

  ofstream right_cam_timestamps_fs;
  ofstream left_cam_timestamps_fs;

  right_cam_timestamps_fs.open(right_cam_timestamps_file.c_str());
  left_cam_timestamps_fs.open(left_cam_timestamps_file.c_str());

  string cpp_file_path = __FILE__;
  string calibration_dir_path = cpp_file_path.substr(0, cpp_file_path.rfind("get_kitti_like_dataset_from_stereo"));
  string left_cam_calibration_file = calibration_dir_path + "../calibrationdata/left_opencv.yaml";
  string right_cam_calibration_file = calibration_dir_path + "../calibrationdata/right_opencv.yaml";

  cv::VideoCapture cap_0("/dev/video0");
  cv::VideoCapture cap_1("/dev/video1");

  cap_0.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
  cap_0.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));
  cap_1.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
  cap_1.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));

  cv::Size image_size;
  image_size.height = 1080;
  image_size.width = 1920;

  // Check if camera opened successfully
  if(!cap_0.isOpened() || !cap_1.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  cv::FileStorage fs(left_cam_calibration_file, cv::FileStorage::READ);
  cv::Mat left_intrinsic_matrix, left_distortion_coeffs;
  fs["camera_matrix"] >> left_intrinsic_matrix;
  fs["distortion_coefficients"] >> left_distortion_coeffs;

  fs.open(right_cam_calibration_file, cv::FileStorage::READ);
  cv::Mat right_intrinsic_matrix, right_distortion_coeffs;
  fs["camera_matrix"] >> right_intrinsic_matrix;
  fs["distortion_coefficients"] >> right_distortion_coeffs;

  fs.release();

  cv::Mat left_undist_map, left_undist_additional_map;

  cv::initUndistortRectifyMap(
    left_intrinsic_matrix,
    left_distortion_coeffs,
    cv::noArray(), //cv::Mat(),
    cv::noArray(), //left_intrinsic_matrix,
    image_size,
    CV_16SC2,
    left_undist_map, left_undist_additional_map
  );

  cv::Mat right_undist_map, right_undist_additional_map;
  cv::initUndistortRectifyMap(
    right_intrinsic_matrix,
    right_distortion_coeffs,
    cv::noArray(), //cv::Mat(),
    cv::noArray(), //right_intrinsic_matrix,
    image_size,
    CV_16SC2,
    right_undist_map, right_undist_additional_map
  );

  cv::Mat left_image, right_image;

  for(int i =0;;i++) {

    // cap_0 >> left_image;
    // auto left_time = std::chrono::high_resolution_clock::now();

    cap_1 >> right_image;
    auto right_time = std::chrono::high_resolution_clock::now();

    // if( !left_image.empty() && !right_image.empty()) {

      // cv::remap(left_image, left_image, left_undist_map, left_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
      //
      // time_t left_time_c = system_clock::to_time_t(left_time);
      // auto left_date_time = std::put_time(std::localtime(&left_time_c), "%F %T");
      // auto left_nanosec = left_time.time_since_epoch();
      //
      // left_cam_timestamps_fs << left_date_time << "." << left_nanosec.count() << std::endl;
      //
      // string left_image_path = left_cam_data_path + to_string(i) + ".png";
      // imwrite(left_image_path, left_image);
      // // cv::imshow("undistorted_left", left_image);


      cv::remap(right_image, right_image, right_undist_map, right_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
      // cv::imshow("undistorted_right", right_image);

      time_t right_time_c = system_clock::to_time_t(right_time);
      auto right_date_time = std::put_time(std::localtime(&right_time_c), "%F %T");
      auto right_nanosec = right_time.time_since_epoch();

      right_cam_timestamps_fs << right_date_time << "." << right_nanosec.count() << std::endl;

      string right_image_path = right_cam_data_path;

      if (i<10) {
        right_image_path += "000" + to_string(i) + ".png";
      }
      else if (i<100) {
        right_image_path += "00" + to_string(i) + ".png";
      }
      else {
        right_image_path += "0" + to_string(i) + ".png";
      }
      cv::Mat right_image_cropped = right_image(cv::Rect(339,353,1242,375));
      cv::imwrite(right_image_path, right_image_cropped);

    // }

    // milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    // milliseconds ms_diff = ms_cur - ms_prev;
    // std::cout << "delay: " << (int)ms_diff.count() << std::endl;

    // Press  ESC on keyboard to exit
    // if((cv::waitKey(10) & 255) == 27) break;
  }




//     std::cout << frame_0.rows << " " << frame_0.cols << std::endl;
//     std::cout << frame_1.rows << " " << frame_1.cols << std::endl;
//
//     // Display the resulting frame
//     namedWindow("video0", WINDOW_NORMAL);
//     resizeWindow("video0", 900, 506);
//     imshow( "video0", frame_0 );
//
//     namedWindow("video1", WINDOW_NORMAL);
//     resizeWindow("video1", 900, 506);
//     imshow( "video1", frame_1 );
//
//     // Press  ESC on keyboard to exit
//     char c=(char)waitKey(25);
//     if(c==27)
//       break;

  cap_0.release();
  cap_1.release();

  cv::destroyAllWindows();
  return 0;
}
