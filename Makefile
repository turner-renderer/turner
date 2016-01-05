CC=c++
CXXFLAGS=-std=c++14 -Wall -O0 -g

renderer: main.o
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf *.o renderer