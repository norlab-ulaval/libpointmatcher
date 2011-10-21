if(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
	include_directories(${GTEST_INCLUDE_DIR})
	add_executable(utest utest.cpp)
	target_link_libraries(utest pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${GTEST_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
	
	add_test(utest ${CMAKE_CURRENT_BINARY_DIR}/utest --path "${CMAKE_SOURCE_DIR}/examples/data/")
else(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)
	message("Google test (gtest) not found - Skipping unit tests")
endif(GTEST_INCLUDE_DIR AND GTEST_LIBRARY)