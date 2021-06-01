CC = g++
CFLAGS = -std=c++11

SOURCES = BallTrackerThread.cpp
OBJECTS = $(SOURCES:.cpp=.o)

OPENCVFLAGS = pkg-config --cflags --libs opencv4

EXECUTABLE=BallTrackerThread



all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) $(CFLAGS) $(OPENCVFLAGS) -o $@

clean:
	-rm -f $(OBJECTS) $(EXECUTABLE)
