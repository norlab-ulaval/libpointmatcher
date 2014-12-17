remake_add_test(
  GOOGLE utest *.cpp ui/*.cpp
  LINK pointmatcher ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}
  COMMAND_ARGS --path ${CMAKE_BINARY_DIR}/test/
)
