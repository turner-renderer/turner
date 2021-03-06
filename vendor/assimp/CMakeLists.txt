cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    assimp
    PREFIX ${CMAKE_BINARY_DIR}/vendor/assimp
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
        -DBUILD_SHARED_LIBS=OFF
        -DASSIMP_BUILD_ASSIMP_TOOLS=OFF
        -DASSIMP_BUILD_SAMPLES=OFF
        -DASSIMP_BUILD_TESTS=OFF
    GIT_REPOSITORY https://github.com/assimp/assimp
    GIT_TAG a5a53433042ae9f9f7d2f9d25312b46fd09702a4
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(assimp install_dir)
set(assimp_INCLUDE_DIR ${install_dir}/include
    CACHE INTERNAL "Path to include folder for Assimp"
)
set(assimp_LIBRARY ${install_dir}/lib/libassimp.a)

# build zlib as static lib from assimp/contrib

ExternalProject_Get_Property(assimp source_dir)

ExternalProject_Add(
    zlibstatic
    DEPENDS assimp
    PREFIX ${CMAKE_BINARY_DIR}/vendor/zlib
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DASSIMP_LIB_INSTALL_DIR=lib
    SOURCE_DIR ${source_dir}/contrib/zlib
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(zlibstatic install_dir)
set(assimp_LIBRARIES ${assimp_LIBRARY} ${install_dir}/lib/libzlibstatic.a
    CACHE INTERNAL "Path to libraries for Assimp"
)
