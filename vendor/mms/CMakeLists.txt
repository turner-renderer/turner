cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    mms
    PREFIX ${CMAKE_BINARY_DIR}/vendor/mms
    GIT_REPOSITORY https://github.com/yandex/mms.git
    GIT_TAG 242bd54
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
)

ExternalProject_Get_Property(mms source_dir)
set(mms_INCLUDE_DIR ${source_dir}/include
    CACHE INTERNAL "Path to include folder for MMS"
)
