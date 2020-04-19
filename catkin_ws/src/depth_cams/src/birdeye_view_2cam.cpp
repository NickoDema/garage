#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

std::vector<cv::Point3d> Generate3DPoints();

int main(int argc, char* argv[]){

  cout << "args:    [board_w] [board_h]" << endl;
  cout << "control: n   - next frame" << endl;
  cout << "         d/u - down/up view" << endl;
  cout << "         esc - exit" << endl;

  string cpp_file_path = __FILE__;
  string calibration_dir_path = cpp_file_path.substr(0, cpp_file_path.rfind("birdeye_view_2cam.cpp"));
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

  //Chessboard params
  int       board_w = atoi(argv[1]);
  int       board_h = atoi(argv[2]);
  int       board_n = board_w * board_h;
  cv::Size  board_sz(board_w, board_h);

  auto chess_finder_params = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS;
  auto sub_pix_terms = cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1);

  double Z = 10;

  cv::Mat left_image, left_gray_image, right_image;

  for(;;) {

    // milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    cap_0 >> left_image;
    // cap_1 >> right_image;

    if( !left_image.empty() ) {
      cv::remap(left_image,
                left_image,
                left_undist_map,
                left_undist_additional_map,
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT,
                cv::Scalar());

      cv::cvtColor( left_image, left_gray_image, cv::COLOR_BGR2GRAY );

      vector<cv::Point2f> corners;
      bool chessboard_found = cv::findChessboardCorners(left_image,
                                                        board_sz,
                                                        corners,
                                                        chess_finder_params
      );

      if( !chessboard_found ) {
          auto right_time = high_resolution_clock::now();
          time_t right_time_c = system_clock::to_time_t(right_time);
          auto right_date_time = std::put_time(std::localtime(&right_time_c), "%T");
          cout << right_date_time << std::endl;

          cv::imshow("undistorted_left", left_image);
          if((cv::waitKey(10) & 255) == 27) break;
          continue;
      }
      // Get Subpixel accuracy on those corners
      // cv::cornerSubPix(
      //     left_gray_image,
      //     corners,
      //     cv::Size(11,11),
      //     cv::Size(-1,-1),
      //     sub_pix_terms
      // );

      // GET THE IMAGE AND OBJECT POINTS:
      // Object points are at (r,c):
      // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1)
      // That means corners are at: corners[r*board_w + c]
      cv::Point2f obj_pts[4], img_pts[4];

      obj_pts[0].x = 0;          obj_pts[0].y = 0;
      obj_pts[1].x = board_w-1;  obj_pts[1].y = 0;
      obj_pts[2].x = 0;          obj_pts[2].y = board_h-1;
      obj_pts[3].x = board_w-1;  obj_pts[3].y = board_h-1;
      img_pts[0] = corners[0];
      img_pts[1] = corners[board_w-1];
      img_pts[2] = corners[(board_h-1)*board_w];
      img_pts[3] = corners[(board_h-1)*board_w + board_w-1];

      cv::circle(left_image, img_pts[0], 9, cv::Scalar(200,   0,   0), 3);
      cv::circle(left_image, img_pts[1], 9, cv::Scalar(0,   200,   0), 3);
      cv::circle(left_image, img_pts[2], 9, cv::Scalar(200, 200,   0), 3);
      cv::circle(left_image, img_pts[3], 9, cv::Scalar(200,   0, 200), 3);

      // cv::drawChessboardCorners( image, board_sz, corners, found );

      cv::Mat H = cv::getPerspectiveTransform( obj_pts, img_pts );

      cv::Mat left_birds_image;

      cv::imshow("undistorted_left", left_image);

      for (;;) {

        H.at<double>(2, 2) = Z;

        cv::warpPerspective(left_image,
                            left_birds_image,
                            H,
                            left_image.size(),
                            cv::WARP_INVERSE_MAP | cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT,
                            cv::Scalar::all(0)
        );

        cv::imshow("left_birds_image", left_birds_image);

        vector<cv::Point2f> image_points;
        vector<cv::Point3f> object_points;
        for( int i=0; i<4; ++i ){
            image_points.push_back( img_pts[i] );
            object_points.push_back(cv::Point3f( obj_pts[i].x, obj_pts[i].y, 0));
        }

        cv::Mat rvec, tvec, rmat;
        cv::solvePnP(object_points,    // 3-d points in object coordinate
                     image_points,     // 2-d points in image coordinates
                     left_intrinsic_matrix,        // Our camera matrix
                     cv::Mat(),        // Since we corrected distortion in the
                                       // beginning,now we have zero distortion
                                       // coefficients
                     rvec,             // Output rotation *vector*.
                     tvec              // Output translation vector.
        );

        cv::Rodrigues( rvec, rmat );

        cout << "____________________________________" << endl;

        auto right_time = high_resolution_clock::now();
        time_t right_time_c = system_clock::to_time_t(right_time);
        auto right_date_time = std::put_time(std::localtime(&right_time_c), "%T");
        auto right_nanosec = right_time.time_since_epoch();

        cout << right_date_time << "." << right_nanosec.count() << std::endl;

        cout << "image_points: "               << image_points << endl;
        cout << "object_points: "              << object_points << endl;
        cout << "rotation matrix: "            << rmat << endl;
        cout << "rotation vector: "            << rvec << endl;
        cout << "translation vector: "         << tvec << endl;
        cout << "homography matrix: "          << H << endl;
        cout << "inverted homography matrix: " << H.inv() << endl;

        int key = 0;
        do {
          key = cv::waitKey() & 255;
      } while (key != 'u' && key != 'd' && key != 'n' && key != 27);

        if(key == 'u') {
            Z += 1.0;
            cout << "Z = " << to_string(Z) << endl;
        }
        if(key == 'd') {
            Z -= 1.0;
            cout << "Z = " << to_string(Z) << endl;
        }
        if(key == 'n') {
            break;
        }
        if(key == 27) {
            return 0;
        }

      }
    }

    // if( !right_image.empty() ) {
    //     cv::remap(right_image, right_image, right_undist_map, right_undist_additional_map, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    //     cv::imshow("undistorted_right", right_image);
    // }
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
