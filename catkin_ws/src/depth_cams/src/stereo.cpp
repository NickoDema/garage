#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <depth_cams/DepthCamsConfig.h>

using namespace std;
using namespace std::chrono;

// std::vector<cv::Point3d> Generate3DPoints();

cv::Ptr<cv::StereoSGBM> sgbm;
int numberOfDisparities;

void dynamic_param_cb(depth_cams::DepthCamsConfig &config, uint32_t level) {

    sgbm = cv::StereoSGBM::create(
      config.min_disparity * 16,
      config.disparity_num * 16,
      config.block_size,
      config.penalty_1,
      config.penalty_2,
      config.disp_12_max_diff,
      config.pre_filter_cap,
      config.uniqueness_ratio,
      config.speckle_window_size,
      config.speckle_range,
      config.sgbm_mode
    );
    numberOfDisparities = config.disparity_num * 16;
}

void get_last_frame(cv::VideoCapture& video, cv::Mat& frame)
{
    //Get total number of frames in the video
    //Won't work on live video capture
    const int frames = video.get(cv::CAP_PROP_FRAME_COUNT);

    //Seek video to last frame
    video.set(cv::CAP_PROP_POS_FRAMES, frames-1);

    //Capture the last frame
    video>>frame;

    //Rewind video
    video.set(cv::CAP_PROP_POS_FRAMES, 0);
}


int main(int argc, char **argv){

  ros::init(argc, argv, "depth_cams");

  string cpp_file_path = __FILE__;
  string calibration_dir_path = cpp_file_path.substr(0, cpp_file_path.rfind("stereo"));
  string left_cam_calibration_file = calibration_dir_path + "../calibrationdata/left_opencv.yaml";
  string right_cam_calibration_file = calibration_dir_path + "../calibrationdata/right_opencv.yaml";

  cv::VideoCapture cap_0("/dev/video0");
  cv::VideoCapture cap_1("/dev/video1");

  cap_0.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
  cap_0.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));
  cap_0.set(cv::CAP_PROP_BUFFERSIZE, 1);

  cap_1.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
  cap_1.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));
  cap_1.set(cv::CAP_PROP_BUFFERSIZE, 1);

  cv::Size image_size;
  image_size.height = 1080;
  image_size.width = 1920;

  // Check if camera opened successfully
  if(!cap_0.isOpened() || !cap_1.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  cv::FileStorage fs(left_cam_calibration_file, cv::FileStorage::READ);
  cv::Mat left_intrinsic_matrix, left_distortion_coeffs, left_rectification_matrix, left_projection_matrix;
  fs["camera_matrix"] >> left_intrinsic_matrix;
  fs["distortion_coefficients"] >> left_distortion_coeffs;
  fs["rectification_matrix"] >> left_rectification_matrix;
  fs["projection_matrix"] >> left_projection_matrix;


  fs.open(right_cam_calibration_file, cv::FileStorage::READ);
  cv::Mat right_intrinsic_matrix, right_distortion_coeffs, right_rectification_matrix, right_projection_matrix;
  fs["camera_matrix"] >> right_intrinsic_matrix;
  fs["distortion_coefficients"] >> right_distortion_coeffs;
  fs["rectification_matrix"] >> right_rectification_matrix;
  fs["projection_matrix"] >> right_projection_matrix;


  fs.release();

  cv::Mat left_undist_map, left_undist_additional_map;
  cv::initUndistortRectifyMap(
    left_intrinsic_matrix,
    left_distortion_coeffs,
    left_rectification_matrix,
    left_projection_matrix,
    image_size,
    CV_16SC2,
    left_undist_map, left_undist_additional_map
  );

  cv::Mat right_undist_map, right_undist_additional_map;
  cv::initUndistortRectifyMap(
    right_intrinsic_matrix,
    right_distortion_coeffs,
    right_rectification_matrix,
    right_projection_matrix,
    image_size,
    CV_16SC2,
    right_undist_map, right_undist_additional_map
  );

  cv::Mat left_image, right_image;

  dynamic_reconfigure::Server<depth_cams::DepthCamsConfig> dynamic_param_srv;
  dynamic_reconfigure::Server<depth_cams::DepthCamsConfig>::CallbackType cb_type;

  cb_type = boost::bind(&dynamic_param_cb, _1, _2);
  dynamic_param_srv.setCallback(cb_type);

  cv::Size new_size(1280,720);

  cv::Mat pair, disp, vdisp;
  // pair.create( image_size.height, image_size.width*2, CV_8UC3 );

  for(;;) {

    // milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    cap_0 >> left_image;
    cap_1 >> right_image;

    // get_last_frame(cap_0, left_image);
    // get_last_frame(cap_1, right_image);

    if( !left_image.empty() ) {
      cv::remap(left_image, left_image, left_undist_map, left_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
      // vector<cv::KeyPoint> keypointsD;
      // auto detector = cv::FastFeatureDetector::create();
      // vector<cv::Mat> descriptors;
      //
      // detector->detect(left_image, keypointsD, cv::Mat());
      // drawKeypoints(left_image, keypointsD, left_image);
      cv::resize(left_image, left_image, new_size);
      cv::namedWindow("undistorted_left", cv::WINDOW_NORMAL);
      cv::resizeWindow("undistorted_left", 768, 432);
      cv::imshow("undistorted_left", left_image);
    }

    if( !right_image.empty() ) {
        cv::remap(right_image, right_image, right_undist_map, right_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        cv::resize(right_image, right_image, new_size);
        // cv::imshow("undistorted_right", right_image);
    }

    if( !left_image.empty() && !right_image.empty() ) {
        sgbm->compute( left_image, right_image, disp);
        // cv::normalize( disp, vdisp, 0, 250, cv::NORM_MINMAX, CV_8U );
        disp.convertTo(vdisp, CV_8U, 255/(numberOfDisparities*16.));
        cv::namedWindow("disparity", cv::WINDOW_NORMAL);
        cv::resizeWindow("disparity", 768, 432);
        cv::imshow( "disparity", vdisp );
    }

    // milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    // milliseconds ms_diff = ms_cur - ms_prev;
    // std::cout << "delay: " << (int)ms_diff.count() << std::endl;

    // Press  ESC on keyboard to exit
    if((cv::waitKey(10) & 255) == 27) break;
    ros::spinOnce();
  }

  cap_0.release();
  cap_1.release();

  cv::destroyAllWindows();
  return 0;
}
