find_package(Python3 COMPONENTS Interpreter REQUIRED)

# detect virtualenv and set Pip args accordingly
if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
  set(_pip_args)
else()
  set(_pip_args "--user")
endif()

execute_process(COMMAND ${Python3_EXECUTABLE} -m pip install -e ${CMAKE_CURRENT_SOURCE_DIR} ${_pip_args})