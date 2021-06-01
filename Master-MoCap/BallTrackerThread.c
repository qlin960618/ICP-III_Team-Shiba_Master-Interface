#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \

( std::ostringstream() << std::dec << x ) ).str()

int main(int argc, char **argv)
{

    ball_0_low = cv::Scalar(66, 179, 101);
    ball_0_high = cv::Scalar(101, 255, 255);
    ball_1_low = cv::Scalar(118, 95, 116);
    ball_1_high = cv::Scalar(189, 255, 255);

    cv::VideoCapture video(0);


    if(!video.isOpened())
    {
        std::cout << "Could not read video file" << endl;
        return 1;
    }

    // Read first frame
    cv::Mat frame;
    cv::Mat frame_blured;
    cv::Mat frame_hsv;
    cv::Mat mask_0;
    cv::Mat mask_1;
    cv::Mat cnts_0;
    cv::Mat cnts_1;

    bool ok = video.read(frame);

    cv::imshow("Tracking", frame);

    while(video.read(frame))
    {
        // Start timer
        double timer = (double)cv::getTickCount();

        cv::GaussianBlur(frame, frame_blured, cv::Size(11, 11), 0);
        cv::cvtColor(frame_blured, frame_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(frame_hsv, ball_0_low, ball_0_high, mask_0);
        cv::inRange(frame_hsv, ball_1_low, ball_1_high, mask_1);

        cv::InputArray element = cv::getStructuringElement( cv::MORPH_RECT,
                      cv::Size( 3, 3 ), cv::Point( 1, 1 ) );

        cv::erode(mask_0, mask_0, element, cv::Point(-1,-1). 2);
        cv::erode(mask_1, mask_1, element, cv::Point(-1,-1). 2);
        cv::dilate(mask_0, mask_0, element, cv::Point(-1,-1). 2);
        cv::dilate(mask_1, mask_1, element, cv::Point(-1,-1). 2);

        std::vector<Vec4i> hierarchy;
        cv::findContours(mask_0, cnts_0, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_1, cnts_1, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Calculate Frames per second (FPS)
        float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

        // if (ok)
        // {
        //     // Tracking success : Draw the tracked object
        //     rectangle(frame, bbox, cv::Scalar( 255, 0, 0 ), 2, 1 );
        // }
        // else
        // {
        //     // Tracking failure detected.
        //     putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        // }
        //
        // // Display tracker type on frame
        // putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
        //
        // Display FPS on frame
        cv::putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);

        // Display frame.
        cv::imshow("Tracking", frame);

        // Exit if ESC pressed.
        int k = cv::waitKey(1);
        if(k == 27)
        {
            break;
        }

    }
}
