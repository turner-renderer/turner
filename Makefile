CXXFLAGS=-std=c++1y -Wall -Wextra -O3 -g -Ivendor/assimp/include
LDFLAGS=-Lvendor/assimp/lib
LDLIBS=-lassimp -lzlibstatic
BINS=test_assimp raycaster raytracer pathtracer radiosity
LIB_OBJS=$(patsubst %.cpp,%.o,$(wildcard lib/*.cpp))

ifdef COVERAGE
CXXFLAGS+=-fprofile-arcs -ftest-coverage
LDFLAGS+=--coverage
endif

all: $(BINS)

$(BINS): CC=$(CXX)
raytracer pathtracer radiosity: LDFLAGS += -Lvendor/docopt
raytracer pathtracer radiosity: LDLIBS += -ldocopt_s -lpthread

test_assimp: test_assimp.o
raycaster: raycaster.o $(LIB_OBJS)
raytracer: main.o raytracer.o $(LIB_OBJS)
pathtracer: main.o pathtracer.o $(LIB_OBJS)
radiosity: radiosity.o lib/types.o $(LIB_OBJS)

main.o radiosity.o: CXXFLAGS += -Ivendor/ThreadPool -Ivendor/docopt -pthread

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
