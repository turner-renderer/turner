#!/bin/sh
# set -ex

mkdir -p build
cd build
cmake .. \
	-DCMAKE_C_COMPILER=clang \
	-DCMAKE_CXX_COMPILER=clang++ \
	-DCMAKE_EXE_LINKER_FLAGS="-lc -lc++ -lc++abi -lunwind"
make
make test
