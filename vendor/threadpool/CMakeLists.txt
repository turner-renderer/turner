cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    threadpool
    PREFIX ${CMAKE_BINARY_DIR}/vendor/threadpool
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    GIT_REPOSITORY https://github.com/progschj/ThreadPool.git
    GIT_TAG 9a42ec1
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
)

ExternalProject_Get_Property(threadpool source_dir)
set(threadpool_INCLUDE_DIR ${source_dir}
	CACHE INTERNAL "Path to include folder for ThreadPool"
)
