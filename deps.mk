.PHONY: build-deps dep-a dep-b assimp catch docopt ThreadPool

build-deps: assimp catch docopt ThreadPool


vendor:
	mkdir -p vendor

vendor/.last-build: deps.mk
	make build-deps
	touch vendor/.last-build


assimp: REPO = https://github.com/assimp/assimp
assimp: COMMIT = 1d8dba1
assimp: ASSIMP_BUILD_OPTS = \
	-DBUILD_SHARED_LIBS=OFF \
	-DASSIMP_BUILD_ASSIMP_TOOLS=OFF \
	-DASSIMP_BUILD_SAMPLES=OFF \
	-DASSIMP_BUILD_TESTS=OFF
assimp: ZLIB_BUILD_OPTIONS = \
	-DASSIMP_LIB_INSTALL_DIR=$$(pwd)/../../lib
vendor/assimp: | vendor
	git clone $(REPO) $@
assimp: vendor/assimp
	cd $< && git fetch && git checkout $(COMMIT)
	cd vendor/assimp/contrib/zlib \
		&& cmake . $(ZLIB_BUILD_OPTIONS) \
		&& make \
		&& make install
	cd vendor/assimp \
		&& cmake . $(ASSIMP_BUILD_OPTS) \
		&& make -j8

catch: REPO = https://github.com/philsquared/Catch
catch: COMMIT = c984fc3
vendor/catch: | vendor
	git clone $(REPO) $@
catch: vendor/catch
	cd $< && git fetch && git checkout $(COMMIT)

docopt: REPO = https://github.com/docopt/docopt.cpp.git
docopt: COMMIT = a4177cc
vendor/docopt: | vendor
	git clone $(REPO) $@
docopt: vendor/docopt
	cd $< && git fetch && git checkout $(COMMIT) \
		&& cmake . && make

ThreadPool: REPO = https://github.com/progschj/ThreadPool.git
ThreadPool: COMMIT = 9a42ec1
vendor/ThreadPool: | vendor
	git clone $(REPO) $@
ThreadPool: vendor/ThreadPool
	cd $< && git fetch && git checkout $(COMMIT)
