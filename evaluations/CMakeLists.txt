add_executable(eval_solution eval_solution.cpp)
target_link_libraries(eval_solution pointmatcher ${NABO_LIBRARY} ${EXTRA_LIBS} ${Boost_LIBRARIES} ncurses)
