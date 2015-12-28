CC=c++
CXXFLAGS=-std=c++1y -Wall -O0 -g

renderer: main.o
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf *.o renderer