#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

std::vector<cv::Point3d> Generate3DPoints();

int main(){

  string cpp_file_path = __FILE__;
  string calibration_dir_path = cpp_file_path.substr(0, cpp_file_path.rfind("video_cap_2cam_undist"));
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

  std::vector<cv::Point3d> objectPoints = Generate3DPoints();

  cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
  rVec.at<double>(0) = 0.0; //-3.9277902400761393e-002;
  rVec.at<double>(1) = 0.0; //3.7803824407602084e-002;
  rVec.at<double>(2) = 0.0; //2.6445674487856268e-002;

  cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
  tVec.at<double>(0) = 0.0;
  tVec.at<double>(1) = 0.0;
  tVec.at<double>(2) = 0.16;

  std::vector<cv::Point2d> projectedPoints;

  cv::projectPoints(objectPoints, rVec, tVec, left_intrinsic_matrix, left_distortion_coeffs, projectedPoints);

  for(;;) {

    milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    cap_0 >> left_image;
    cap_1 >> right_image;

    if( !left_image.empty() ) {


    for(int i=0; i <= projectedPoints.size(); i++) {
        cv::circle(left_image, projectedPoints[i], 7, cv::Scalar(250,150,150), -1);
    }

      cv::remap(left_image, left_image, left_undist_map, left_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
      cv::imshow("undistorted_left", left_image);
    }

    if( !right_image.empty() ) {
        cv::remap(right_image, right_image, right_undist_map, right_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        // cv::imshow("undistorted_right", right_image);
    }

    milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    milliseconds ms_diff = ms_cur - ms_prev;
    std::cout << "delay: " << (int)ms_diff.count() << std::endl;

    // Press  ESC on keyboard to exit
    if((cv::waitKey(10) & 255) == 27) break;
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


std::vector<cv::Point3d> Generate3DPoints()
{
    std::vector<cv::Point3d> points;
    points.push_back(cv::Point3d( .015,  .015,  .015));
    points.push_back(cv::Point3d(-.015,  .015,  .015));
    points.push_back(cv::Point3d(-.015, -.015,  .015));
    points.push_back(cv::Point3d( .015, -.015,  .015));
    points.push_back(cv::Point3d( .015,  .015, -.015));
    points.push_back(cv::Point3d(-.015,  .015, -.015));
    points.push_back(cv::Point3d(-.015, -.015, -.015));
    points.push_back(cv::Point3d( .015, -.015, -.015));

    return points;
}
