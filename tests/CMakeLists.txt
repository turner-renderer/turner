cmake_minimum_required(VERSION 3.2)

add_subdirectory(${EXT_PROJECTS_DIR}/catch ${CMAKE_CURRENT_BINARY_DIR}/catch)
include_directories(${catch_INCLUDE_DIR})

add_library(catch_main OBJECT catch_main.cpp)
add_dependencies(catch_main catch)

set(TESTS
    test_algorithm
    test_clipping
    test_config
    test_effects
    test_functional
    test_intersection
    test_kdtree
    test_lambertian
    test_mesh
    test_progress_bar
    test_radiosity
    test_raster
    test_sampling
    test_triangle
    test_types
)

foreach (TEST_NAME ${TESTS})
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp
    	$<TARGET_OBJECTS:catch_main> $<TARGET_OBJECTS:common>
    )
    add_test(${TEST_NAME} ${TEST_NAME})
endforeach ()

target_link_libraries(test_config ${docopt_LIBRARIES})
target_link_libraries(test_mesh ${openmesh_LIBRARIES})
target_link_libraries(test_radiosity ${openmesh_LIBRARIES})
