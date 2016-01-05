CC=c++
CXXFLAGS=-std=c++1y -Wall -O0 -g -Ilib/assimp/include
LDFLAGS=-Llib/assimp/lib -lassimp

all: renderer test_assimp

renderer: main.o
	$(CXX) $(CXXFLAGS) $< -o $@


bootstrap:
	git submodule init
	git submodule update
	cd lib/assimp && cmake CMakeLists.txt
	make -C lib/assimp

clean:
	rm -rf *.o renderer test_assimp *.dSYM genfiles

distclean: clean
	cd lib/assimp && git clean -df && git reset --hard && rm -r CMakeCache.txt

.PHONY: all clean bootstrap


TESTS = \
	test_intersection

include tests/tests.mk
