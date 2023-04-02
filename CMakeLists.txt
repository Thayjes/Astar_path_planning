# OffWorld Copyright 2022
# CMakeLists build file template

cmake_minimum_required(VERSION 3.14)

# CPP version requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



# Project configuration
set(PROJECT_NAME
path_planning
)

# project version
project(${PROJECT_NAME} VERSION 1.0)
set(${PROJECT_NAME}_VERSION 1.0)

# Function to prepend the subdirectory to source files in subdirectories
FUNCTION(PREPEND var )
   SET(listVar "")
   FOREACH(f ${${var}})
       LIST(APPEND listVar "${CMAKE_CURRENT_SOURCE_DIR}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

# After a normal build, we can specify the location of various outputs of the
# build. We put executables and static libraries outside the build directory in
# bin/ and lib/, respectively.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/lib")

# find packages
# find_package(<package_name> <version> REQUIRED)
find_package(Python3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(OpenCV REQUIRED)


# Source configuration
# Include source code and headers. This calls the CMakeLists.txt in each
# subdirectory. These can define their own libraries, executables, etc. as targets, 
# but here we define all exportable targets in the root CMakeLists.txt.
add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(scripts)
enable_testing()
add_subdirectory(tests)

# Add the library ${PROJECT_NAME} as a target, with the contents of src/ and include/ as dependencies
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_INC})
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})

# These variables slightly modify the install location to allow for version
# specific installations.
set(${PROJECT_NAME}_INCLUDE_DEST "include/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION}")
set(${PROJECT_NAME}_LIB_DEST "lib/${PROJECT_NAME}-${${PROJECT_NAME}_VERSION}")


target_include_directories(${PROJECT_NAME} PUBLIC
   # headers to include when building from source
   $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
   $<BUILD_INTERFACE:${${PROJECT_NAME}_DIR}/include> 

   # headers to include when installing  (implicitly prefixes with ${CMAKE_INSTALL_PREFIX}).
   $<INSTALL_INTERFACE:include> 
   )


# Install ${PROJECT_NAME} in CMAKE_INSTALL_PREFIX (defaults to /usr/local on linux). 
# To change the install location, run 
#   cmake -DCMAKE_INSTALL_PREFIX=<desired-install-path> ..

# install(...) specifies installation rules for the project. It can specify
# location of installed files on the system, user permissions, build
# configurations, etc. Here, we are only copying files.
# install(TARGETS ...) specifies rules for installing targets. 
# Here, we are taking a target or list of targets (${PROJECT_NAME}) and telling CMake
# the following:
#   - put shared libraries associated with ${PROJECT_NAME} in ${${PROJECT_NAME}_LIB_DEST}
#   - put static libraries associated with ${PROJECT_NAME} in ${${PROJECT_NAME}_LIB_DEST}
#   - put include files associated with ${PROJECT_NAME} in ${${PROJECT_NAME}_INCLUDE_DEST}
# We also need to specify the export that is associated with ${PROJECT_NAME}; an export 
# is just a list of targets to be installed.
# So we are associating ${PROJECT_NAME} with ${PROJECT_NAME}Targets.
install(
    # targets to install
    TARGETS ${PROJECT_NAME} 
    # name of the CMake "export group" containing the targets we want to install
    EXPORT ${PROJECT_NAME}Targets
    # Dynamic, static library and include destination locations after running
    # "make install"
    LIBRARY DESTINATION ${${PROJECT_NAME}_LIB_DEST}
    ARCHIVE DESTINATION ${${PROJECT_NAME}_LIB_DEST} 
    INCLUDES DESTINATION ${${PROJECT_NAME}_INCLUDE_DEST}
    )

# We now need to install the export ${PROJECT_NAME}Targets that we defined above. This
# is needed in order for another project to import ${PROJECT_NAME} using 
#   find_package(${PROJECT_NAME})
# find_package(${PROJECT_NAME}) will look for ${PROJECT_NAME}-config.cmake to provide
# information about the targets contained in the project ${PROJECT_NAME}. Fortunately,
# this is specified in the export ${PROJECT_NAME}Targets, so we will install this too.
# install(EXPORT ...) will install the information about an export. Here, we
# save it to a file {$${PROJECT_NAME}_LIB_DEST}/${PROJECT_NAME}Targets.cmake and prepend 
# everything inside ${PROJECT_NAME}Targets  with the namespace ${PROJECT_NAME}::.
install(
    # The export we want to save (matches name defined above containing the
    # install targets)
    EXPORT ${PROJECT_NAME}Targets
    # CMake file in which to store the export's information
    FILE  ${PROJECT_NAME}Targets.cmake
    # Namespace prepends all targets in the export (when we import later, we
    # will use ${PROJECT_NAME}::${PROJECT_NAME})
    NAMESPACE ${PROJECT_NAME}::
    # where to place the resulting file (here, we're putting it with the library)
    DESTINATION ${${PROJECT_NAME}_LIB_DEST}
    )

# install(FILES ...) simply puts files in a certain place with certain
# properties. We're just copying include files to the desired include directory
# here.
install(FILES ${${PROJECT_NAME}_INC} DESTINATION ${${PROJECT_NAME}_INCLUDE_DEST})

# Write a "version file" in case someone wants to only load a particular version of
# ${PROJECT_NAME} 
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${${PROJECT_NAME}_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# Copies the resulting CMake config files to the installed library directory
install(
    FILES 
        "cmake/${PROJECT_NAME}-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${${PROJECT_NAME}_LIB_DEST}
    )

#########
# GTest #
#########
include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
#   URL https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip
  URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
