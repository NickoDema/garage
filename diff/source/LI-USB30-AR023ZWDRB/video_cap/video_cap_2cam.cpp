#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;

int main(){

  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap_0("/dev/video0");
  VideoCapture cap_1("/dev/video1");

  // Check if camera opened successfully
  if(!cap_0.isOpened() || !cap_1.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

  while(1){

    Mat frame_0;
    Mat frame_1;

    // Capture frame-by-frame
    cap_0 >> frame_0;
    cap_1 >> frame_1;

    // If the frame is empty, break immediately
    if (frame_0.empty() || frame_1.empty()) {
      cout << "Some frame is empty" << endl;
      break;
    }

    // Display the resulting frame
    namedWindow("video0", WINDOW_NORMAL);
    resizeWindow("video0", 900, 506);
    imshow( "video0", frame_0 );

    namedWindow("video1", WINDOW_NORMAL);
    resizeWindow("video1", 900, 506);
    imshow( "video1", frame_1 );

    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;

    milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    milliseconds ms_diff = ms_cur - ms_prev;
    float fps = 1000.0/(float)ms_diff.count();
    std::cout << "fps: " << fps << std::endl;
    ms_prev = ms_cur;
  }

  // When everything done, release the video capture object
  cap_0.release();
  cap_1.release();

  // Closes all the frames
  destroyAllWindows();

  return 0;
}
