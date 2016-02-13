CC=$(CXX)
CXXFLAGS=-std=c++1y -Wall -Wextra -O3 -g -Ivendor/assimp/include
LDFLAGS=-Lvendor/assimp/lib -lassimp -lzlibstatic
BINS=renderer test_assimp raycaster raytracer pathtracer

all: $(BINS)

raytracer pathtracer: CXXFLAGS += -Ivendor/docopt.cpp
raytracer pathtracer: LDFLAGS += -Lvendor/docopt.cpp -ldocopt_s

raycaster: raycaster.o lib/types.o
raytracer: main.o raytracer.o lib/types.o
pathtracer: main.o pathtracer.o lib/types.o

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
	rm -rf *.o lib/*.o test_assimp *.dSYM genfiles $(BINS)

distclean: clean
	cd vendor/assimp && git clean -df && git reset --hard
	cd vendor/docopt.cpp && git clean -df && git reset --hard

.PHONY: all clean distclean bootstrap


TESTS = \
	test_intersection \
	test_lambertian \
	test_types \
	stress_test_kdtree

#%: %.o lib/*.h
#	$(CXX) $(LDFLAGS) -o $@ $<

include tests/tests.mk
