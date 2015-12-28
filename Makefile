CC=c++
CXXFLAGS=-std=c++1y -Wall -O0 -g

renderer: main.cpp
	$(CXX) $(LDFLAGS) -o renderer main.cpp

all: renderer
