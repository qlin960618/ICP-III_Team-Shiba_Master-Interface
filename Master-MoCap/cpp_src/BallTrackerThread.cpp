#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <chrono>
#include <thread>



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

bool send_udp(int srcPort, int destPort, const void * packet_data, int p_size){
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((unsigned short)destPort);

    int sent_bytes = sendto(srcPort, (const char*)packet_data,
                        p_size, 0, (sockaddr*)&addr,
                        sizeof(sockaddr_in));
    if(sent_bytes != p_size)
    {
        return false;
    }
    return true;
}

int recv_udp(int srcPort, void * packet_data, int max_packet_size){
    sockaddr_in from;
    socklen_t fromLength = sizeof(from);

    int byte=recvfrom(srcPort, (char*)packet_data, max_packet_size,
                0, (sockaddr*)&from, &fromLength);

    return byte;
}

const cv::String argKeys=
    "{help h usage ?    |       | print this message}"
    "{@vid              |0      | Specify video source}"
    "{width             |1280   | Video resolution width}"
    "{height            |720    | Video resolution height}"
    "{rPort             |2220   | Specify UDP listening port}"
    "{sPort             |1003   | Specify UDP targeting port}"
    "{show              |0      | specify if want to show display}"
    ;
////argument format
int main(int argc, char **argv)
{
    ///parsing arguments
    cv::CommandLineParser parser(argc, argv, argKeys);

    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }

    int vidSrc = parser.get<int>(0);
    char winTitle[20];
    std::sprintf(winTitle, "Camera: %d", vidSrc);
    int recvPort = parser.get<int>("rPort");
    int sendPort = parser.get<int>("sPort");
    int tVidWidth = parser.get<int>("width");
    int tVidHeight = parser.get<int>("height");


    bool show_REALTIME=false;
    if(parser.get<int>("show")>0)
    {
        std::cout<< "Camera: " << vidSrc << " enable display" <<std::endl;
         show_REALTIME= true;
    }

    ////setting default color
    auto ball_0_low = cv::Scalar(66, 179, 101);
    auto ball_0_high = cv::Scalar(101, 255, 255);
    auto ball_1_low = cv::Scalar(118, 95, 116);
    auto ball_1_high = cv::Scalar(189, 255, 255);

    //////////////////////OPENING SOCKET
    int hSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (hSock<=0){
        std::cout << "failled to open socket" << std::endl;
        return 0;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons((unsigned short)recvPort);

    if( bind(hSock,
        (const sockaddr*) &address,
        sizeof(sockaddr_in))<0 ){

            std::cout << "Failled to bind Socket" <<std::endl;
            return 0;
        }
    // std::cout << "socket opened" << std::endl;
    // int nonBlocking=0;
    // if(fcntl(hSock, F_SETFL, O_NONBLOCK, nonBlocking) == -1){
    //     std::cout << "Failed to set non-blocking" << std::endl;
    // }
    //creating sent buffer
    char sendBuff[100];
    char recvBuff[100];
    //////////////////////OPENING SOCKET

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
    int len = std::sprintf(sendBuff, "w:%d,%d", vidWidth, vidHeight);
    send_udp(hSock, sendPort, sendBuff, len);

    if(!video.isOpened())
    {
        std::cout << "Could not read camera:" << vidSrc << std::endl;
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

    std::cout << "loop begin" << std::endl;
    while(video.read(frame))
    {
        ball0_detec=0;
        ball1_detec=0;
        // Start timer

        //Listen for command and start processing
        int len = recv_udp(hSock, recvBuff, 100);
        if(len>0){
            if (recvBuff[0]=='c' && len>=14){ //set filter Color
                std::cout << "set filter: ";
                for(int k=0; k<12; k++)
                    std::cout << (int)(uint8_t)recvBuff[k+2] << ",";
                std::cout << std::endl;
                ball_0_low =   cv::Scalar(uint8_t(recvBuff[2]), uint8_t(recvBuff[3]), uint8_t(recvBuff[4]));
                ball_0_high =  cv::Scalar(uint8_t(recvBuff[5]), uint8_t(recvBuff[6]), uint8_t(recvBuff[7]));
                ball_1_low =   cv::Scalar(uint8_t(recvBuff[8]), uint8_t(recvBuff[9]), uint8_t(recvBuff[10]));
                ball_1_high =  cv::Scalar(uint8_t(recvBuff[11]), uint8_t(recvBuff[12]), uint8_t(recvBuff[13]));
                continue;
            }else if(recvBuff[0]=='e'){ //exit program
                std::cout << "exit"<<std::endl;
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
        len = std::sprintf(sendBuff, "d:%d:%d,%d,%d,%d,%d,%d", vidSrc,
                    ball0_detec, ball0_posX, ball0_posY,
                    ball1_detec, ball1_posX, ball1_posY);
        send_udp(hSock, sendPort, sendBuff, len);

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
    std::cout << "exiting from cpp Backend" <<std::endl;
    //program exit or error
    len = std::sprintf(sendBuff, "err:%d", vidSrc);
    send_udp(hSock, sendPort, sendBuff, len);
}
