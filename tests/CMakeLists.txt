add_unit_test(test_cop_stabilizer cpp/test_cop_stabilizer.cpp)
target_link_libraries(test_cop_stabilizer PUBLIC ${PROJECT_NAME})
target_include_directories(test_cop_stabilizer
                           PRIVATE Boost::unit_test_framework)
if(BUILD_PYTHON_INTERFACE)
  add_python_unit_test("test_cop_stabilizer_py"
                       "tests/python/test_cop_stabilizer.py" "python")
endif()
