cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_NAMESPACE gepetto)
set(PROJECT_NAME biped-stabilizer)
set(PROJECT_DESCRIPTION "Stabilizer for Biped Locomotion")
set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
IF(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  IF(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    MESSAGE(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
  ELSE()
    include(FetchContent)
    FetchContent_Declare("jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  ENDIF()
ENDIF()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/boost.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
find_package(Eigen3 REQUIRED)
find_package(roscpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

if(BUILD_PYTHON_INTERFACE)
  find_package(eigenpy 2.7.10 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
endif()

# Main Library
set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/cop_stabilizer.hpp
  )
set(${PROJECT_NAME}_SOURCES
  src/cop_stabilizer.cpp
  )
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${roscpp_LIBRARIES})
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> ${roscpp_INCLUDE_DIR} ${roscpp_INCLUDE_DIRS})
if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif()


# Python Bindings
if(BUILD_PYTHON_INTERFACE)
  set(${PY_NAME}_SOURCES
    python/${PROJECT_NAME}.cpp
    python/cop_stabilizer.cpp
    python/eigen_isometry_3d.cpp
  )
  add_library(${PY_NAME}_py SHARED ${${PY_NAME}_SOURCES})
  target_link_libraries(${PY_NAME}_py PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
  target_include_directories(${PY_NAME}_py SYSTEM PRIVATE ${PYTHON_INCLUDE_DIR})
  set_target_properties(${PY_NAME}_py PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../.."
    LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
    OUTPUT_NAME ${PY_NAME}_cpp
    PREFIX ""
    SUFFIX ${PYTHON_EXT_SUFFIX}
  )
  TARGET_LINK_BOOST_PYTHON(${PY_NAME}_py PRIVATE)
  install(TARGETS ${PY_NAME}_py DESTINATION ${${PY_NAME}_INSTALL_DIR})
  install(DIRECTORY python/biped_stabilizer DESTINATION ${${PY_NAME}_INSTALL_DIR}/..)
endif()

# Unit tests
ADD_UNIT_TEST(test_cop_stabilizer tests/cpp/test_cop_stabilizer.cpp)
target_link_libraries(test_cop_stabilizer PUBLIC ${PROJECT_NAME})
target_include_directories(test_cop_stabilizer PRIVATE Boost::unit_test_framework)
if(BUILD_PYTHON_INTERFACE)
  ADD_PYTHON_UNIT_TEST("test_cop_stabilizer_py" "tests/python/test_cop_stabilizer.py" "python")
endif()


# Installation
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
