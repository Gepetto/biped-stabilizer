if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_unit_test(test-biped-stabilizer-cpp cpp/test_cop_stabilizer.cpp)
  target_link_libraries(test-biped-stabilizer-cpp PUBLIC ${PROJECT_NAME})
  target_include_directories(test-biped-stabilizer-cpp PRIVATE doctest::doctest)
endif()

if(BUILD_PYTHON_INTERFACE)
  add_python_unit_test("test-biped-stabilizer-py"
                       "tests/python/test_cop_stabilizer.py" "python")
endif()
