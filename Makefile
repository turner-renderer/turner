CXXFLAGS=-std=c++1y -Wall -Wextra -O0 -g -Ivendor/assimp/include
LDFLAGS=-Lvendor/assimp/lib
LDLIBS=-lassimp -lzlibstatic
BINS=renderer test_assimp raycaster raytracer pathtracer

ifdef COVERAGE
CXXFLAGS+=-fprofile-arcs -ftest-coverage
LDFLAGS+=--coverage
endif

all: $(BINS)

$(BINS): CC=$(CXX)
raytracer pathtracer: LDFLAGS += -Lvendor/docopt.cpp
raytracer pathtracer: LDLIBS += -ldocopt_s -lpthread

renderer: renderer.o
test_assimp: test_assimp.o
raycaster: raycaster.o lib/types.o lib/triangle.o
raytracer: main.o raytracer.o lib/types.o lib/triangle.o
pathtracer: main.o pathtracer.o lib/types.o lib/triangle.o

main.o: CXXFLAGS += -Ivendor/ThreadPool -Ivendor/docopt.cpp -pthread

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
		&& make -j8
	cd vendor/docopt.cpp \
		&& cmake . \
		&& make

clean:
	rm -rf *.o lib/*.o test_assimp *.dSYM *.gcov *.gcno *.gcda genfiles $(BINS)

distclean: clean
	cd vendor/assimp && git clean -df && git reset --hard
	cd vendor/docopt.cpp && git clean -df && git reset --hard

.PHONY: all clean distclean bootstrap


TESTS = \
	test_intersection \
	test_lambertian \
	test_types \
	test_sampling \
	test_config \
	test_kdtree


include tests/tests.mk
