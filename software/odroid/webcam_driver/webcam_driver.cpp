/****
Copies images from a webcam and broadcasts them to LCM
leveraging OpenCV's nice webcam drivers.

MIT Hyperloop 2016 -- gizatt

I'm working from this guy's nice starting code to get going
https://wimsworld.wordpress.com/2013/07/19/webcam-on-beagleboardblack-using-opencv/
****/

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <unistd.h> // for sleep
#include<opencv2/opencv.hpp>

#include "common_utils.h"
#include <lcmtypes/bot_core_image_t.h>

#define MITHL_CAMERA_JPEG_QUALITY 45

using namespace std;
using namespace cv;

std::string timeToISO8601(const time_t & TheTime)
{
    std::ostringstream ISOTime;
    struct tm * UTC = gmtime(&TheTime);
    ISOTime.fill('0');
    ISOTime << UTC->tm_year+1900 << "-";
    ISOTime.width(2);
    ISOTime << UTC->tm_mon+1 << "-";
    ISOTime.width(2);
    ISOTime << UTC->tm_mday << "T";
    ISOTime.width(2);
    ISOTime << UTC->tm_hour << ":";
    ISOTime.width(2);
    ISOTime << UTC->tm_min << ":";
    ISOTime.width(2);
    ISOTime << UTC->tm_sec;
    ISOTime << "Z";
    return(ISOTime.str());
}
std::string getTimeISO8601(void)
{
    time_t timer;
    time(&timer);
    return(timeToISO8601(timer));
}

void *lcmMonitor(void *plcm) {
  lcm_t *lcm = (lcm_t *) plcm;
  while (1)
    lcm_handle(lcm);
}

int main(int argc, char ** argv)
{
    lcm_t * lcm = lcm_create("udpm://239.255.76.67:62237?ttl=0");
    pthread_t lcmThread;
    pthread_create(&lcmThread, NULL, lcmMonitor, lcm);

    char * channel = "WEBCAM";
    int num = -1;
    if (argc >= 2) num = atoi(argv[1]);
    if (argc >= 3) channel = argv[2];
    VideoCapture capture(num);   // Using -1 tells OpenCV to grab whatever camera is available.
    if(!capture.isOpened()){
        std::cout << "Failed to connect to the camera." << std::endl;
        return(1);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH,320);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,270);

    double last_send_time = getUnixTime();
    while (1) {
        // wasteful but lower latency. todo: see if this is troublesome on the odroid.
        Mat C920Image;
        capture >> C920Image;
        if (getUnixTime() - last_send_time > 0.5){
            last_send_time = getUnixTime();
            if(!C920Image.empty())
            {
/*
                line(C920Image, Point(0, C920Image.rows/2), Point(C920Image.cols, C920Image.rows/2), Scalar(255, 255, 255, 32)); // Horizontal line at center
                line(C920Image, Point(C920Image.cols/2, 0), Point(C920Image.cols/2, C920Image.rows), Scalar(255, 255, 255, 32)); // Vertical line at center

                circle(C920Image, Point(C920Image.cols/2, C920Image.rows/2), 240, Scalar(255, 255, 255, 32)); // Circles based at center
                putText(C920Image, "10", Point((C920Image.cols/2 + 240), (C920Image.rows/2)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255));
                circle(C920Image, Point(C920Image.cols/2, C920Image.rows/2), 495, Scalar(255, 255, 255, 32)); // Circles based at center
                putText(C920Image, "20", Point((C920Image.cols/2 + 495), (C920Image.rows/2)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255));
                circle(C920Image, Point(C920Image.cols/2, C920Image.rows/2), 785, Scalar(255, 255, 255, 32)); // Circles based at center
                putText(C920Image, "30", Point((C920Image.cols/2 + 785), (C920Image.rows/2)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255));
                circle(C920Image, Point(C920Image.cols/2, C920Image.rows/2), 1141, Scalar(255, 255, 255, 32)); // Circles based at center
                putText(C920Image, "40", Point((C920Image.cols/2 + 1141), (C920Image.rows/2)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255));

*/
                string DateTimeText = getTimeISO8601();
                int baseline=0;
                Size textSize = getTextSize(DateTimeText, FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);

                //putText(C920Image, DateTimeText, Point((C920Image.cols - textSize.width), (C920Image.rows - baseline)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255));

                // prepare compression params
                vector<int> compression_params;
                compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                compression_params.push_back(MITHL_CAMERA_JPEG_QUALITY);

                // do some compression
                vector<uchar> buf;
                bool success = imencode(".jpg", C920Image, buf, compression_params);

                // LCM encode and publish
                bot_core_image_t imagemsg;
                imagemsg.utime = getUnixTime() * 1000 * 1000;
                imagemsg.width = C920Image.cols;
                imagemsg.height = C920Image.rows;
                imagemsg.row_stride = 0;
                imagemsg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
                imagemsg.size = sizeof(uchar) * buf.size();
                imagemsg.data = &buf[0];
                imagemsg.nmetadata = 0;
                bot_core_image_t_publish(lcm, channel, &imagemsg);

                std::cout << DateTimeText << " Captured and published. " << std::endl;
            }
            std::cout << getTimeISO8601() << "\r" << std::flush;
        }
    }
    return 0;
}
