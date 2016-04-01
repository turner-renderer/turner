CXXFLAGS=-std=c++1y -Wall -Wextra -O3 -g -Ivendor/assimp/include
LDFLAGS=-Lvendor/assimp/lib
LDLIBS=-lassimp -lzlibstatic
BINS=test_assimp raycaster raytracer pathtracer

ifdef COVERAGE
CXXFLAGS+=-fprofile-arcs -ftest-coverage
LDFLAGS+=--coverage
endif

all: $(BINS)

$(BINS): CC=$(CXX)
raytracer pathtracer: LDFLAGS += -Lvendor/docopt
raytracer pathtracer: LDLIBS += -ldocopt_s -lpthread

test_assimp: test_assimp.o
raycaster: raycaster.o lib/types.o lib/triangle.o
raytracer: main.o raytracer.o lib/types.o lib/triangle.o
pathtracer: main.o pathtracer.o lib/types.o lib/triangle.o

main.o: CXXFLAGS += -Ivendor/ThreadPool -Ivendor/docopt -pthread

bootstrap: vendor/.last-build
# run bootstrap before building anything
*.o **/*.o: | bootstrap

clean:
	rm -rf *.o lib/*.o test_assimp *.dSYM *.gcov *.gcno *.gcda genfiles $(BINS)

distclean: clean
	rm -rf vendor

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
