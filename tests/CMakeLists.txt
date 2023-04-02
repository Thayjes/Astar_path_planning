
# OffWorld Copyright 2022
# CMakeLists build file template

cmake_minimum_required(VERSION 3.14)
# CPP version requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
# Explicitly list the test source code and headers.
set(${PROJECT_NAME}_TEST_SRC
    test_gtest_demo.cpp
)
set(${PROJECT_NAME}_TEST_HEADER
    test_gtest_demo.h
)

# Now simply link against gtest or gtest_main as needed.
add_executable(test_gtest_demo ${${PROJECT_NAME}_TEST_SRC} ${${PROJECT_NAME}_TEST_HEADER})
target_include_directories(test_gtest_demo PUBLIC
   $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include> 
   )
target_link_libraries(test_gtest_demo gtest_main ${OpenCV_LIBRARIES} GTest::gmock_main)
add_test(NAME addition_test COMMAND test_gtest_demo)

# Add custom command post build, this ensures tests are run after compilation
add_custom_command(
    TARGET test_gtest_demo
    COMMENT "Run tests"
    POST_BUILD
    COMMAND test_gtest_demo
    COMMAND pytest ${CMAKE_CURRENT_SOURCE_DIR}
)