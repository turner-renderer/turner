cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    cereal
    PREFIX ${CMAKE_BINARY_DIR}/vendor/cereal
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    GIT_REPOSITORY https://github.com/USCiLab/cereal.git
    GIT_TAG v1.2.2
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
)

ExternalProject_Get_Property(cereal source_dir)
set(cereal_INCLUDE_DIR ${source_dir}/include
    CACHE INTERNAL "Path to include folder for Cereal"
)
