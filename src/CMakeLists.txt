add_library(vvc SHARED encoder.cpp decoder.cpp)
target_link_libraries(vvc ${PCL_LIBRARIES})
