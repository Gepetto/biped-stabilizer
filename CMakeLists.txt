cmake_minimum_required(VERSION 3.22.1)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME biped-stabilizer)
set(PROJECT_DESCRIPTION "Stabilizer for Biped Locomotion")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})

# Check if the submodule cmake have been initialized
find_package(jrl-cmakemodules QUIET CONFIG)
if(jrl-cmakemodules_FOUND)
  message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  get_property(
    JRL_CMAKE_MODULES
    TARGET jrl-cmakemodules::jrl-cmakemodules
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
elseif(EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  message(STATUS "JRL cmakemodules found in the source tree.")
  set(JRL_CMAKE_MODULES "${CMAKE_SOURCE_DIR}/cmake")
else()
  message(STATUS "JRL cmakemodules not found. Let's fetch it.")
  include(FetchContent)
  FetchContent_Declare(
    "jrl-cmakemodules"
    GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
  FetchContent_MakeAvailable("jrl-cmakemodules")
  FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
find_package(Eigen3 REQUIRED)

if(BUILD_PYTHON_INTERFACE)
  find_package(eigenpy 2.7.12 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

if(BUILD_TESTING)
  # Prepare doctest for other targets to use.
  find_package(doctest QUIET)
  if(NOT doctest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      doctest
      GIT_REPOSITORY https://github.com/doctest/doctest.git
      GIT_TAG master)
    FetchContent_MakeAvailable(doctest)
  endif()
  # Add extra dependencies in case the python interface is built.
  if(BUILD_PYTHON_INTERFACE)
    find_package(pinocchio REQUIRED)
    find_package(example-robot-data REQUIRED)
  endif()
endif()

# Third party dependencies
set(THIRD_PARTY_HEADERS
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_algorithm.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_axis_projection_descriptor.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_clipping.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_duplicates.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_earclipping.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_graphics_net.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_graphics_opengl.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_graphics_vcl.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_gui.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_hull.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_instantiate.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_math.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_matrix.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_matrix.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_minimum_bounding_ball.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_naive_group_intersections.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_nd.hpp
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_nd.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_normalization.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_ordered_polygon.inl
    include/${PROJECT_NAME}/third_party/wykobi/wykobi_utilities.hpp)

# Main Library
set(${PROJECT_NAME}_HEADERS include/${PROJECT_NAME}/cop_stabilizer.hpp
                            ${THIRD_PARTY_HEADERS})
set(${PROJECT_NAME}_SOURCES src/cop_stabilizer.cpp)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()

# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif()

# Unit tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# Installation
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)

  install(
    FILES include/${PROJECT_NAME}/third_party/wykobi/wykobi_license.txt
    DESTINATION include/${PROJECT_NAME}/third_party/wykobi/wykobi_license.txt)
endif()
