cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    docopt
    PREFIX ${CMAKE_BINARY_DIR}/vendor/docopt
    GIT_REPOSITORY https://github.com/docopt/docopt.cpp.git
    GIT_TAG 1811022
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
        -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
        -DCMAKE_INSTALL_LIBDIR:PATH=lib
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(docopt install_dir)
set(docopt_INCLUDE_DIR ${install_dir}/include
	CACHE INTERNAL "Path to include folder for Docopt"
)
set(docopt_LIBRARIES ${install_dir}/lib/libdocopt.a
	CACHE INTERNAL "Path to libraries for Docopt"
)
