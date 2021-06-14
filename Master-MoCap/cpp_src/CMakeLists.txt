cmake_minimum_required(VERSION 2.8)
project( BallTrackerThread)
set("OpenCV_DIR" "C:/lib/build_opencv")
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( BallTrackerThread BallTrackerThread.cpp )
target_link_libraries( BallTrackerThread ${OpenCV_LIBS} )