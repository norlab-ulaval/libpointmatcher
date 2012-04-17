add_executable(evaluate_dataset evaluate_dataset.cpp)
target_link_libraries(evaluate_dataset pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES})

add_executable(generate_random_poses generate_random_poses.cpp)
target_link_libraries(generate_random_poses ${EXTRA_LIBS} ${Boost_LIBRARIES})
