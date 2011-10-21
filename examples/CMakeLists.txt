add_executable(icp icp.cpp)
target_link_libraries(icp pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(icp_simple icp_simple.cpp)
target_link_libraries(icp_simple pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_THREAD_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

add_executable(align_sequence align_sequence.cpp)
target_link_libraries(align_sequence pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

add_executable(convertCSVtoVTK convertCSVtoVTK.cpp)
target_link_libraries(convertCSVtoVTK pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

add_executable(list_modules list_modules.cpp)
target_link_libraries(list_modules pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_THREAD_LIBRARY})
