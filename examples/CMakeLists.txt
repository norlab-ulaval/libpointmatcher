add_executable(pmicp icp.cpp)
target_link_libraries(pmicp pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})
install(TARGETS pmicp RUNTIME DESTINATION bin)

add_executable(icp_simple icp_simple.cpp)
target_link_libraries(icp_simple pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})

add_executable(align_sequence align_sequence.cpp)
target_link_libraries(align_sequence pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})

add_executable(convertCSVtoVTK convertCSVtoVTK.cpp)
target_link_libraries(convertCSVtoVTK pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})

add_executable(list_modules list_modules.cpp)
target_link_libraries(list_modules pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})

add_executable(build_map build_map.cpp)
target_link_libraries(build_map pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})
