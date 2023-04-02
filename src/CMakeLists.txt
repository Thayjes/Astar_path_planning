# OffWorld Copyright 2022
# CMakeLists build file template

# Make an explicit list of all source files in `${PROJECT_NAME}_SRC`. This is important
# because CMake is not a build system: it is a build system generator. Suppose
# you add a file foo.cpp to src/ after running cmake .. . If you set
# `${PROJECT_NAME}_SRC` with `file(GLOB ... )`, this is not passed to the makefile;
# the makefile doesn't know that foo.cpp exists and will not re-run cmake. Your
# collaborator's builds will fail and it will be unclear why. Whether you use
# file(GLOB ...) or not, you will need to re-run cmake, but with an explicit
# file list, you know beforehand why your code isn't compiling.
cmake_minimum_required(VERSION 3.14)

# CPP version requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


set(${PROJECT_NAME}_SRC
    plan.cpp
    format.cc
    )

# Form the full path to the source files...
PREPEND(${PROJECT_NAME}_SRC)
# ... and pass the variable to the parent scope.
set(${PROJECT_NAME}_SRC ${${PROJECT_NAME}_SRC}  PARENT_SCOPE)
