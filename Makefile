CC=c++
CXXFLAGS=-std=c++1y -Wall -O0 -g -Ilib/assimp/include
LDFLAGS=-Llib/assimp/lib -lassimp -lzlibstatic

all: renderer test_assimp raycaster

renderer: main.o
	$(CXX) $(CXXFLAGS) $< -o $@


bootstrap:
	git submodule init
	git submodule update
	cd lib/assimp && cmake CMakeLists.txt -DBUILD_SHARED_LIBS=OFF
	cd lib/assimp/contrib/zlib \
		&& cmake . -DASSIMP_LIB_INSTALL_DIR=$(shell pwd)/../../lib
	make -C lib/assimp/contrib/zlib
	make -C lib/assimp/contrib/zlib install

clean:
	rm -rf *.o renderer test_assimp *.dSYM genfiles raycaster

distclean: clean
	cd lib/assimp && git clean -df && git reset --hard && rm -r CMakeCache.txt

.PHONY: all clean bootstrap


TESTS = \
	test_intersection

include tests/tests.mk
