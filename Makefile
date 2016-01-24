CC=c++
CXXFLAGS=-std=c++1y -Wall -O0 -g -Ivendor/assimp/include -Ivendor/docopt.cpp
LDFLAGS=-Lvendor/assimp/lib -Lvendor/docopt.cpp -lassimp -lzlibstatic -ldocopt_s

all: renderer test_assimp raycaster raytracer

bootstrap:
	git submodule init
	git submodule update
	cd vendor/assimp && cmake CMakeLists.txt -DBUILD_SHARED_LIBS=OFF
	cd vendor/assimp/contrib/zlib \
		&& cmake . -DASSIMP_LIB_INSTALL_DIR=$$(pwd)/../../lib
	make -C vendor/assimp/contrib/zlib
	make -C vendor/assimp/contrib/zlib install
	make -C vendor/assimp
	make -C vendor/assimp install
	cd vendor/docopt.cpp && cmake . && make

clean:
	rm -rf *.o renderer test_assimp *.dSYM genfiles raycaster raytracer

distclean: clean
	cd vendor/assimp && git clean -df && git reset --hard
	cd vendor/docopt.cpp && git clean -df && git reset --hard

.PHONY: all clean distclean bootstrap


TESTS = \
	test_intersection \
	test_lambertian

include tests/tests.mk
