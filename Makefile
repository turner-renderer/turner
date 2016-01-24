CC=c++
CXXFLAGS=-std=c++1y -Wall -O0 -g -Ivendor/assimp/include
LDFLAGS=-Lvendor/assimp/lib -lassimp -lzlibstatic

all: renderer test_assimp raycaster raytracer

bootstrap:
	git submodule init
	git submodule update
	cd vendor/assimp && cmake CMakeLists.txt -DBUILD_SHARED_LIBS=OFF
	cd vendor/assimp/contrib/zlib \
		&& cmake . -DASSIMP_LIB_INSTALL_DIR=$$(pwd)/../../lib
	make -C vendor/assimp/contrib/zlib
	make -C vendor/assimp/contrib/zlib install

clean:
	rm -rf *.o renderer test_assimp *.dSYM genfiles raycaster raytracer

distclean: clean
	cd vendor/assimp && git clean -df && git reset --hard && rm -r CMakeCache.txt

.PHONY: all clean distclean bootstrap


TESTS = \
	test_intersection \
	test_lambertian

include tests/tests.mk
