#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <sys/types.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
#include <chrono>
#include <thread>



// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

#define DEFAULT_VIDEO_WIDTH 1280
#define DEFAULT_VIDEO_HEIGHT 720


#define DISABLE_HELP_MSG


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


const cv::String argKeys=
    "{help h usage ?    |       | print this message}"
    "{@vid              |0      | Specify video source}"
    "{width             |1280   | Video resolution width}"
    "{height            |720    | Video resolution height}"
    "{show              |0      | specify if want to show display}"
    ;
////argument format
int main(int argc, char **argv)
{
    ///parsing arguments
    cv::CommandLineParser parser(argc, argv, argKeys);

#ifndef DISABLE_HELP_MSG
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }
#endif

    int vidSrc = parser.get<int>(0);
    char winTitle[20];
    std::sprintf(winTitle, "Camera: %d", vidSrc);
    int tVidWidth = parser.get<int>("width");
    int tVidHeight = parser.get<int>("height");


    bool show_REALTIME=false;
    if(parser.get<int>("show")>0)
    {
        std::cout<< "stat:Camera: " << vidSrc << " enable display" <<std::endl;
         show_REALTIME= true;
    }

    ////setting default color
    auto ball_0_low = cv::Scalar(66, 179, 101);
    auto ball_0_high = cv::Scalar(101, 255, 255);
    auto ball_1_low = cv::Scalar(118, 95, 116);
    auto ball_1_high = cv::Scalar(189, 255, 255);

    //creating sent/recv buffer
    char sendBuff[100];
    char recvBuff[100];

    ////set delay for some time to let python be ready
    std::chrono::milliseconds timespan(500);
    std::this_thread::sleep_for(timespan);

    ///OPEN CV intialization
    cv::VideoCapture video(vidSrc);
    video.set(cv::CAP_PROP_FRAME_WIDTH, tVidWidth);
    video.set(cv::CAP_PROP_FRAME_HEIGHT, tVidHeight);

    int vidWidth = video.get(cv::CAP_PROP_FRAME_WIDTH);
    int vidHeight = video.get(cv::CAP_PROP_FRAME_HEIGHT);

    // outStr << vidWidth << "," << vidHeight << "\n";
    std::cout << "stat:w"<<vidWidth<<","<<vidHeight<<std::endl;


    if(!video.isOpened())
    {
        std::cout << "erro:Could not read camera:" << vidSrc << std::endl;
        return 1;
    }

    // Read first frame
    cv::Mat frame;
    cv::Mat frame_blured;
    cv::Mat frame_hsv;
    cv::Mat mask_0;
    cv::Mat mask_1;
    std::vector<std::vector<cv::Point> > cnts_0;
    std::vector<std::vector<cv::Point> > cnts_1;

    bool ok = video.read(frame);

    if(show_REALTIME){
        cv::imshow(winTitle, frame);
    }

    bool ball0_detec=0;
    int ball0_posX=0;
    int ball0_posY=0;
    bool ball1_detec=0;
    int ball1_posX=0;
    int ball1_posY=0;

    auto tStart = std::chrono::high_resolution_clock::now();

    std::cout << "stat:loop begin" << std::endl;
    while(video.read(frame))
    {
        ball0_detec=0;
        ball1_detec=0;
        // Start timer

        //Listen for command and start processing
        std::cin >> recvBuff;
        int len = std::strlen(recvBuff);
        if(len>0){
            if (recvBuff[0]=='c' && len>=14){ //set filter Color
                std::cout << "stat:set filter: ";
                for(int k=0; k<12; k++)
                    std::cout << (int)(uint8_t)recvBuff[k+2] << ",";
                std::cout << std::endl;
                ball_0_low =   cv::Scalar(uint8_t(recvBuff[2]), uint8_t(recvBuff[3]), uint8_t(recvBuff[4]));
                ball_0_high =  cv::Scalar(uint8_t(recvBuff[5]), uint8_t(recvBuff[6]), uint8_t(recvBuff[7]));
                ball_1_low =   cv::Scalar(uint8_t(recvBuff[8]), uint8_t(recvBuff[9]), uint8_t(recvBuff[10]));
                ball_1_high =  cv::Scalar(uint8_t(recvBuff[11]), uint8_t(recvBuff[12]), uint8_t(recvBuff[13]));
                continue;
            }else if(recvBuff[0]=='e'){ //exit program
                std::cout << "stat:exit"<<std::endl;
                break;
            }else if (recvBuff[0]=='n'){ //process next frame
                // std::cout << "next Frame"<<std::endl;
            }else{
                // std::cout << "wrong cmd" << std::endl;
                continue;
            }
        }


        cv::GaussianBlur(frame, frame_blured, cv::Size(11, 11), 0);
        cv::cvtColor(frame_blured, frame_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(frame_hsv, ball_0_low, ball_0_high, mask_0);
        cv::inRange(frame_hsv, ball_1_low, ball_1_high, mask_1);


        cv::erode(mask_0, mask_0, cv::Mat(), cv::Point(-1, -1), 2);
        cv::erode(mask_1, mask_1, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask_0, mask_0, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask_1, mask_1, cv::Mat(), cv::Point(-1, -1), 2);

        cv::findContours(mask_0, cnts_0, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(mask_1, cnts_1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if(cnts_0.size()>0){
            ball0_detec = 1;    //enable detection check
            int id0 = getMaxAreaCountourID(cnts_0);

            auto M0 = cv::moments(cnts_0.at(id0));

            ball0_posX = M0.m10 / M0.m00;
            ball0_posY = M0.m01 / M0.m00;

            if(show_REALTIME){
                cv::Point2f ball0_center;
                float r0;
                cv::minEnclosingCircle(cnts_0.at(id0), ball0_center, r0);
                cv::circle(frame, ball0_center, r0, cv::Scalar(0,255,255), 2);
                cv::circle(frame, cv::Point(ball0_posX, ball0_posY), 5, cv::Scalar(255,0,0), -1);
            }

        }
        if(cnts_1.size()>0){
            ball1_detec = 1;    //enable detection check
            int id1 = getMaxAreaCountourID(cnts_1);

            auto M1 = cv::moments(cnts_1.at(id1));

            ball1_posX = M1.m10 / M1.m00;
            ball1_posY = M1.m01 / M1.m00;

            if(show_REALTIME){
                cv::Point2f ball1_center;
                float r1;
                cv::minEnclosingCircle(cnts_1.at(id1), ball1_center, r1);
                cv::circle(frame, ball1_center, r1, cv::Scalar(0,255,255), 2);
                cv::circle(frame, cv::Point(ball1_posX, ball1_posY), 5, cv::Scalar(255,0,0), -1);
            }
        }

        //sent data to python frontend
        len = std::sprintf(sendBuff, "data:%d,%d,%d,%d,%d,%d\n",
                    ball0_detec, ball0_posX, ball0_posY,
                    ball1_detec, ball1_posX, ball1_posY);
        std::cout << sendBuff;

        auto tEnd = std::chrono::high_resolution_clock::now();
        // Calculate Frames per second (FPS)
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tEnd - tStart);
        float fps =  1000000.0 / duration.count();
        tStart = tEnd;

        if (show_REALTIME){
            // Display FPS on frame
            cv::putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255,0,0), 2);

            // Display frame.
            cv::resize(frame, frame, cv::Size(DEFAULT_VIDEO_WIDTH/2, DEFAULT_VIDEO_HEIGHT/2));
            cv::imshow(winTitle, frame);
            // Exit if ESC pressed.
            int k = cv::waitKey(1);
            if(k == 27)
                break;
        }
    }
    //program exit or error
    std::cout << "stat:exiting from cpp Backend" <<std::endl;
}
