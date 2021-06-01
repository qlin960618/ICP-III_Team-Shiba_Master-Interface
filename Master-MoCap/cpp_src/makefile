CC = g++
CFLAGS = -std=c++11

SOURCES = BallTrackerThread.cpp
OBJECTS = $(SOURCES:.cpp=.o)

OPENCV = $(shell pkg-config opencv4 --cflags --libs)
# LIB = $(OPENCV)
LIB = -I/usr/include/opencv -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_datasets -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_line_descriptor -lopencv_optflow -lopencv_video -lopencv_plot -lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_rgbd -lopencv_viz -lopencv_surface_matching -lopencv_text -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml -lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core


EXECUTABLE=../BallTrackerThread



all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS)  $(LIB) $(CFLAGS) -o $@

clean:
	-rm -f $(OBJECTS) $(EXECUTABLE)
