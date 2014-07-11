# External libraries packed with libpointmatcher

# GTest
add_library(gtest gtest/gtest-all.cc)

# Wrapped yaml-cpp
if(NOT USE_SYSTEM_YAML_CPP)
add_subdirectory(yaml-cpp-pm)
endif()
