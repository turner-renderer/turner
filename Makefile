CC=$(CXX)
CXXFLAGS=-std=c++1y -Wall -Wextra -O3 -g -Ivendor/assimp/include
LDFLAGS=-Lvendor/assimp/lib -lassimp -lzlibstatic
BINS=renderer test_assimp raycaster raytracer pathtracer

all: $(BINS)

raytracer pathtracer: LDFLAGS += -Lvendor/docopt.cpp -ldocopt_s

raycaster: raycaster.o lib/types.o
raytracer: main.o raytracer.o lib/types.o
pathtracer: main.o pathtracer.o lib/types.o

main.o: CXXFLAGS += -Ivendor/ThreadPool -Ivendor/docopt.cpp

bootstrap: ASSIMP_BUILD_OPTS = \
	-DBUILD_SHARED_LIBS=OFF \
	-DASSIMP_BUILD_ASSIMP_TOOLS=OFF \
	-DASSIMP_BUILD_SAMPLES=OFF \
	-DASSIMP_BUILD_TESTS=OFF
bootstrap: ZLIB_BUILD_OPTIONS = \
	-DASSIMP_LIB_INSTALL_DIR=$$(pwd)/../../lib
bootstrap:
	git submodule init
	git submodule update
	cd vendor/assimp/contrib/zlib \
		&& cmake . $(ZLIB_BUILD_OPTIONS) \
		&& make \
		&& make install
	cd vendor/assimp \
		&& cmake . $(ASSIMP_BUILD_OPTS) \
		&& make
	cd vendor/docopt.cpp \
		&& cmake . \
		&& make

clean:
	rm -rf *.o lib/*.o test_assimp *.dSYM genfiles $(BINS)

distclean: clean
	cd vendor/assimp && git clean -df && git reset --hard
	cd vendor/docopt.cpp && git clean -df && git reset --hard

.PHONY: all clean distclean bootstrap


TESTS = \
	test_intersection \
	test_lambertian \
	test_types \
	stress_test_kdtree \
	test_sampling


include tests/tests.mk
