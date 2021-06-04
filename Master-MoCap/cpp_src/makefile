CC = g++
CFLAGS = -std=c++11

SOURCES = BallTrackerThread.cpp
OBJECTS = $(SOURCES:.cpp=.o)

OPENCV = $(shell pkg-config opencv4 --cflags --libs)
LIB = $(OPENCV)


EXECUTABLE=../BallTrackerThread



all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS)  $(LIB) $(CFLAGS) -o $@

clean:
	-rm -f $(OBJECTS) $(EXECUTABLE)
