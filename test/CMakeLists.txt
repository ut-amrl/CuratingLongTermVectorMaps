FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest.a pthread glog gtest_main.a X11 ${CERES_LIBRARIES})
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(stf_filter_tests stf_filter_test.cc)
  catkin_add_gtest(line_extraction_test line_extraction_test.cc
                                      ../src/line_extraction.cc)
  target_link_libraries(stf_filter_tests ${catkin_LIBRARIES})
  target_link_libraries(line_extraction_test ${catkin_LIBRARIES})
  add_dependencies(stf_filter_tests ${catkin_EXPORTED_TARGETS})
  add_dependencies(line_extraction_tests ${catkin_EXPORTED_TARGETS})
else()
  package_add_test(stf_filter_tests stf_filter_test.cc)
  package_add_test(line_extraction_test line_extraction_test.cc
                                      ../src/line_extraction.cc)
endif()
