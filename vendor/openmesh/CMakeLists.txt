cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    openmesh
    PREFIX ${CMAKE_BINARY_DIR}/vendor/openmesh
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DBUILD_APPS=OFF
        -DOPENMESH_BUILD_SHARED=OFF
    GIT_REPOSITORY https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
    GIT_TAG OpenMesh-6.3
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
    PATCH_COMMAND
        patch -p1 < ${CMAKE_SOURCE_DIR}/vendor/openmesh/001-fix-include-of-struct-timeval-definition.patch
)

ExternalProject_Get_Property(openmesh install_dir)
set(openmesh_INCLUDE_DIR ${install_dir}/include
    CACHE INTERNAL "Path to include folder for OpenMesh"
)
set(openmesh_LIBRARIES ${install_dir}/lib/libOpenMeshCore.a
    CACHE INTERNAL "Path to libraries for OpenMesh")
