cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    catch
    PREFIX ${CMAKE_BINARY_DIR}/vendor/catch
    GIT_REPOSITORY https://github.com/philsquared/Catch.git
    GIT_TAG v1.5.1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)

ExternalProject_Get_Property(catch source_dir)
set(catch_INCLUDE_DIR ${source_dir}/include
	CACHE INTERNAL "Path to include folder for Catch"
)
