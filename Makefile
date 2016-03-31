CXXFLAGS=-std=c++1y -Wall -Wextra -O3 -g -Ivendor/assimp/include
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

test_assimp: test_assimp.o
raycaster: raycaster.o lib/types.o lib/triangle.o
raytracer: main.o raytracer.o lib/types.o lib/triangle.o
pathtracer: main.o pathtracer.o lib/types.o lib/triangle.o

main.o: CXXFLAGS += -Ivendor/ThreadPool -Ivendor/docopt.cpp -pthread

bootstrap: build-deps

clean:
	rm -rf *.o lib/*.o test_assimp *.dSYM *.gcov *.gcno *.gcda genfiles $(BINS)

distclean: clean
	cd vendor/assimp && git clean -df && git reset --hard
	cd vendor/docopt.cpp && git clean -df && git reset --hard

.PHONY: all clean distclean bootstrap


TESTS = \
	test_intersection \
	test_clipping \
	test_lambertian \
	test_types \
	test_sampling \
	test_config \
	test_kdtree


include tests/tests.mk
include deps.mk
