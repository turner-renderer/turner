cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    openmesh
    PREFIX ${CMAKE_BINARY_DIR}/vendor/openmesh
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    GIT_REPOSITORY https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git
    GIT_TAG OpenMesh-6.3
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(openmesh install_dir)
set(openmesh_INCLUDE_DIR ${install_dir}/include
    CACHE INTERNAL "Path to include folder for OpenMesh"
)
set(openmesh_LIBRARIES ${install_dir}/lib/libOpenMeshCore.a
    CACHE INTERNAL "Path to libraries for OpenMesh")
