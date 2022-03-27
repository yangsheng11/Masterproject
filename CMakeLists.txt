cmake_minimum_required(VERSION 3.10)

project(WholeBodyMotion)

# version number
set(${PROJECT_NAME}_MAJOR_VERSION 1)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/etc/CMakeModules)

find_package(Simox REQUIRED)
find_package(MMMCore QUIET)
find_package(MMMTools QUIET)
find_package(drake QUIET)
# Installation directories (can be overwritten by the user)
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR share/CMake/${PROJECT_NAME} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# cmake 3.10 does not understand c++2a, so we tell it we will handle the standard flag
set(CMAKE_CXX_STANDARD_DEFAULT)
add_definitions(-std=c++2a)

# Compile flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -Wpedantic -Wno-long-long -Wno-unused-parameter -Wno-maybe-uninitialized -Wno-deprecated-declarations ")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs ")


# Set directory
set(PROJECT_DATA_DIR "${PROJECT_SOURCE_DIR}/data/")
add_definitions(-DPROJECT_DATA_DIR="${PROJECT_DATA_DIR}" -D_SCL_SECURE_NO_WARNINGS)
set(PROJECT_SOURCE_DIR "${PROJECT_SOURCE_DIR}/source")

# Add custom path for find scripts
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

# Enable testing (for ctest / make test)
enable_testing()

# Put binaries into lib/ and bin/ folders
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# set MOTION_HANDLER_PLUGIN_LIB_DIR (search directory for motion handler plugins)
set(MOTION_HANDLER_PLUGIN_LIB_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/motionHandlerPluginLibs)

# set MOTION_HANDLER_PLUGIN_LIB_DIR (search directory for motion plot plugins)
set(MOTION_PLOT_PLUGIN_LIB_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/motionPlotPluginLibs)


# set SENSOR_PLOT_PLUGIN_LIB_DIR (search directory for sensor plot plugins)
set(SENSOR_PLUGIN_LIB_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sensorPluginLibs)

# set SENSOR_VISUALISATION_PLUGIN_LIB_DIR (search directory for sensorvisualisation plugins)
set(SENSOR_VISUALISATION_PLUGIN_LIB_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sensorVisualisationPluginLibs)

add_subdirectory(source)

# Add all targets to the build-tree export set
#export(TARGETS ${PROJECT_NAME} FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
export(TARGETS FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")

# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)

# ... for both
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Install the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion.cmake
install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev
)

# Install the export set for use with the install-tree
#install(EXPORT ${PROJECT_NAME}Targets DESTINATION
#    "${INSTALL_CMAKE_DIR}" COMPONENT dev
#)

add_subdirectory(applications)
