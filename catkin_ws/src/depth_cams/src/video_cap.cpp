#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;

int main(){

  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap("/dev/video0");

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  milliseconds ms_prev = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());

  while(1){

    Mat frame;
    // Capture frame-by-frame
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty())
      break;

    // Display the resulting frame
    imshow( "Frame", frame );

    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;

    milliseconds ms_cur = std::chrono::duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    milliseconds ms_diff = ms_cur - ms_prev;
    float fps = 1000.0/(float)ms_diff.count();
    // std::cout << fps << std::endl;
    ms_prev = ms_cur;
  }

  // When everything done, release the video capture object
  cap.release();

  // Closes all the frames
  destroyAllWindows();

  return 0;
}
