cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)

ExternalProject_Add(
    eigen
    PREFIX ${CMAKE_BINARY_DIR}/vendor/eigen
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    HG_REPOSITORY https://bitbucket.org/eigen/eigen
    HG_TAG 3.2.7
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(eigen install_dir)
set(eigen_INCLUDE_DIR ${install_dir}/include/eigen3
    CACHE INTERNAL "Path to include folder for Eigen"
)
