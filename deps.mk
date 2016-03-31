.PHONY: build-deps dep-a dep-b

build-deps: assimp catch docopt ThreadPool


vendor:
	mkdir -p vendor

vendor/.last-build: deps.mk
	make build-deps
	touch vendor/.last-build


assimp: COMMIT = 1d8dba1
assimp: ASSIMP_BUILD_OPTS = \
	-DBUILD_SHARED_LIBS=OFF \
	-DASSIMP_BUILD_ASSIMP_TOOLS=OFF \
	-DASSIMP_BUILD_SAMPLES=OFF \
	-DASSIMP_BUILD_TESTS=OFF
assimp: ZLIB_BUILD_OPTIONS = \
	-DASSIMP_LIB_INSTALL_DIR=$$(pwd)/../../lib
vendor/assimp: vendor
	git clone https://github.com/assimp/assimp vendor/assimp
assimp: vendor/assimp
	cd vendor/assimp \
		&& git fetch \
        && git checkout $(COMMIT) \
	cd vendor/assimp/contrib/zlib \
		&& cmake . $(ZLIB_BUILD_OPTIONS) \
		&& make \
		&& make install
	cd vendor/assimp \
		&& cmake . $(ASSIMP_BUILD_OPTS) \
		&& make -j8

catch: COMMIT = c984fc3
vendor/catch: vendor
	git clone https://github.com/philsquared/Catch vendor/catch
catch: vendor/catch
	cd vendor/catch \
		&& git fetch \
		&& git checkout $(COMMIT)

docopt: COMMIT = a4177cc
vendor/docopt: vendor
	git clone https://github.com/docopt/docopt.cpp.git vendor/docopt
docopt: vendor/docopt
	cd vendor/docopt \
		&& git fetch \
		&& git checkout $(COMMIT) \
		&& cmake . \
		&& make

ThreadPool: COMMIT = 9a42ec1
vendor/ThreadPool: vendor
	git clone https://github.com/progschj/ThreadPool.git vendor/ThreadPool
ThreadPool: vendor/ThreadPool
	cd vendor/ThreadPool \
		&& git fetch \
		&& git checkout $(COMMIT)
