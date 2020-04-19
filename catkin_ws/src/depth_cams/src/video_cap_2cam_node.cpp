#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"


using namespace std;

std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}


void convert_frame_to_message(const cv::Mat & frame, std::string frame_id, uint seq, ros::Time time, sensor_msgs::Image & msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = frame_id;
  msg.header.stamp = time;
  msg.header.seq = seq;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "stereo");
    ros::NodeHandle nh;

    ros::Publisher video0_pub = nh.advertise<sensor_msgs::Image>("left/image_raw", 1);
    ros::Publisher video1_pub = nh.advertise<sensor_msgs::Image>("right/image_raw", 1);

    ros::Rate loop_rate(30);
    ros::Time time;

    cv::VideoCapture cap_0("/dev/video0");
    cv::VideoCapture cap_1("/dev/video1");

    cap_0.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
    cap_0.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));
    cap_1.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(1920));
    cap_1.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(1080));

    if(!cap_0.isOpened() || !cap_1.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    cv::Mat frame_0;
    cv::Mat frame_1;

    int count = 0;
    while(ros::ok()){

        cap_0 >> frame_0;
        cap_1 >> frame_1;

        if (frame_0.empty() || frame_1.empty()) {
            cout << "Some frame is empty" << endl;
            break;
        }

        sensor_msgs::Image msg0;
        sensor_msgs::Image msg1;


        time = ros::Time::now();


        convert_frame_to_message(frame_0, "camera_0", count, time, msg0);
        convert_frame_to_message(frame_1, "camera_1", count, time, msg1);

        video0_pub.publish(msg0);
        video1_pub.publish(msg1);

        ++count;
        ros::spinOnce();
        loop_rate.sleep();
    }

    cap_0.release();
    cap_1.release();

    cv::destroyAllWindows();

    return 0;
}




// --------------------------------------------------------------------------

// #include <chrono>

    // milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

// // Display the resulting frame
// cv::namedWindow("video0", WINDOW_NORMAL);
// cv::resizeWindow("video0", 900, 506);
// cv::imshow( "video0", frame_0 );
//
// cv::namedWindow("video1", WINDOW_NORMAL);
// cv::resizeWindow("video1", 900, 506);
// cv::imshow( "video1", frame_1 );

// Press  ESC on keyboard to exit
// char c=(char)cv::waitKey(25);
// if(c==27)
//   break;

// milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
// milliseconds ms_diff = ms_cur - ms_prev;
// float fps = 1000.0/(float)ms_diff.count();
// std::cout << "fps: " << fps << std::endl;
// ms_prev = ms_cur;
