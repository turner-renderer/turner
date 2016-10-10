cmake_minimum_required(VERSION 3.2)
set_directory_properties(PROPERTIES CLEAN_NO_CUSTOM 1)
include(ExternalProject)
find_package(Git REQUIRED)

ExternalProject_Add(
    docopt
    PREFIX ${CMAKE_BINARY_DIR}/vendor/docopt
    GIT_REPOSITORY https://github.com/docopt/docopt.cpp.git
    GIT_TAG a4177cc
    CMAKE_ARGS
    	-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
    	-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    LOG_DOWNLOAD ON
    UPDATE_COMMAND ""
)

ExternalProject_Get_Property(docopt install_dir)
set(docopt_INCLUDE_DIR ${install_dir}/include
	CACHE INTERNAL "Path to include folder for Docopt"
)
set(docopt_LIBRARIES ${install_dir}/lib/libdocopt_s.a
	CACHE INTERNAL "Path to libraries for Docopt"
)
