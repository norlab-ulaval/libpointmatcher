enable_testing()

include_directories(../contrib/gtest)
add_executable(utest utest.cpp)
target_link_libraries(utest gtest pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${GTEST_BOTH_LIBRARIES} ${Boost_LIBRARIES})
add_test(utest ${CMAKE_CURRENT_BINARY_DIR}/utest --gtest_output=xml:${CMAKE_CURRENT_BINARY_DIR}/test_results.xml --path "${CMAKE_SOURCE_DIR}/examples/data/")

if (UNIX)
	configure_file(CTestCustom.cmake ${CMAKE_BINARY_DIR})
endif (UNIX)