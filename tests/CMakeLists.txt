cmake_minimum_required(VERSION 3.2)

add_subdirectory(${EXT_PROJECTS_DIR}/catch ${CMAKE_CURRENT_BINARY_DIR}/catch)
include_directories(${catch_INCLUDE_DIR})

add_library(catch_main OBJECT catch_main.cpp)
add_dependencies(catch_main catch)

set(TESTS
    test_clipping
    test_config
    test_gauss_seidel
    test_intersection
    test_kdtree
    test_lambertian
    test_sampling
    test_types
    test_radiosity
    test_effects
    test_solid_angle
    test_algorithm
    test_raster
    test_triangle
    test_incidence
)

foreach (TEST_NAME ${TESTS})
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp
    	$<TARGET_OBJECTS:catch_main> $<TARGET_OBJECTS:common>
    )
    add_test(${TEST_NAME} ${TEST_NAME})
endforeach ()
