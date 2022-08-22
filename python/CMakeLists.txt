set(${PY_NAME}_SOURCES biped_stabilizer.cpp cop_stabilizer.cpp
                       eigen_isometry_3d.cpp)
add_library(${PY_NAME} SHARED ${${PY_NAME}_SOURCES})
target_link_libraries(${PY_NAME} PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
target_include_directories(${PY_NAME} SYSTEM PRIVATE ${PYTHON_INCLUDE_DIR})
set_target_properties(
  ${PY_NAME}
  PROPERTIES INSTALL_RPATH "$ORIGIN/../../.."
             LIBRARY_OUTPUT_DIRECTORY ${PY_NAME}
             OUTPUT_NAME ${PY_NAME}_cpp
             PREFIX ""
             SUFFIX ${PYTHON_EXT_SUFFIX})

set(PYTHON_SOURCES __init__.py)

foreach(source ${PYTHON_SOURCES})
  python_install_on_site(${PY_NAME} ${source})
endforeach(source)
