#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>

// using namespace cv;
// using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

#define DEFAULT_VIDEO_WIDTH 1280
#define DEFAULT_VIDEO_HEIGHT 720



int getMaxAreaCountourID(std::vector<std::vector<cv::Point>> contours){
    double maxArea = 0;
    int maxAreaCountourID = -1;
    for(int j=0; j < contours.size(); j++){
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea){
            maxArea = newArea;
            maxAreaCountourID = j;
        }
    }
    return maxAreaCountourID;
}


int main(int argc, char **argv)
{
    bool show_REALTIME = false;

    auto ball_0_low = cv::Scalar(66, 179, 101);
    auto ball_0_high = cv::Scalar(101, 255, 255);
    auto ball_1_low = cv::Scalar(118, 95, 116);
    auto ball_1_high = cv::Scalar(189, 255, 255);

    cv::VideoCapture video(0);

    video.set(cv::CAP_PROP_FRAME_WIDTH, DEFAULT_VIDEO_WIDTH);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, DEFAULT_VIDEO_HEIGHT);

    if(!video.isOpened())
    {
        std::cout << "Could not read video file" << std::endl;
        return 1;
    }

    // Read first frame
    cv::Mat frame;
    cv::Mat frame_blured;
    cv::Mat frame_hsv;
    cv::Mat mask_0;
    cv::Mat mask_1;
    // cv::OutputArrayOfArrays cnts_0;
    // cv::OutputArrayOfArrays cnts_1;
    std::vector<std::vector<cv::Point> > cnts_0;
    std::vector<std::vector<cv::Point> > cnts_1;

    bool ok = video.read(frame);

    if(show_REALTIME){
        cv::imshow("Tracking", frame);
    }

    while(video.read(frame))
    {
        // Start timer
        double timer = (double)cv::getTickCount();

        cv::GaussianBlur(frame, frame_blured, cv::Size(11, 11), 0);
        cv::cvtColor(frame_blured, frame_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(frame_hsv, ball_0_low, ball_0_high, mask_0);
        cv::inRange(frame_hsv, ball_1_low, ball_1_high, mask_1);


        cv::erode(mask_0, mask_0, cv::Mat(), cv::Point(-1, -1), 2);
        cv::erode(mask_1, mask_1, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask_0, mask_0, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask_1, mask_1, cv::Mat(), cv::Point(-1, -1), 2);

        cv::findContours(mask_0.clone(), cnts_0, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_1.clone(), cnts_1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if(cnts_0.size()>0){
            int id0 = getMaxAreaCountourID(cnts_0);

            auto M0 = cv::moments(cnts_0.at(id0));

            int ball0_posX = M0.m10 / M0.m00;
            int ball0_posY = M0.m01 / M0.m00;

            if(show_REALTIME){
                cv::Point2f ball0_center;
                float r0;
                cv::minEnclosingCircle(cnts_0.at(id0), ball0_center, r0);

                cv::circle(frame, ball0_center, r0, cv::Scalar(0,255,255), 2);
                cv::circle(frame, cv::Point(ball0_posX, ball0_posY), 5, cv::Scalar(255,0,0), -1);
            }

        }
        if(cnts_1.size()>0){
            int id1 = getMaxAreaCountourID(cnts_1);

            auto M1 = cv::moments(cnts_1.at(id1));

            int ball1_posX = M1.m10 / M1.m00;
            int ball1_posY = M1.m01 / M1.m00;

            if(show_REALTIME){
                cv::Point2f ball1_center;
                float r1;
                cv::minEnclosingCircle(cnts_1.at(id1), ball1_center, r1);

                cv::circle(frame, ball1_center, r1, cv::Scalar(0,255,255), 2);
                cv::circle(frame, cv::Point(ball1_posX, ball1_posY), 5, cv::Scalar(255,0,0), -1);
            }
        }


        double timer_end = (double)cv::getTickCount();
        // Calculate Frames per second (FPS)
        float fps = cv::getTickFrequency() / (timer_end - timer);
        timer_end = timer;

        if (show_REALTIME){
            // Display FPS on frame
            cv::putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255,0,0), 2);

            // Display frame.
            cv::resize(frame, frame, cv::Size(DEFAULT_VIDEO_WIDTH/2, DEFAULT_VIDEO_HEIGHT/2));
            cv::imshow("Tracking", frame);

            // Exit if ESC pressed.
            int k = cv::waitKey(1);
            if(k == 27)
            {
                break;
            }
        }

    }
}
