add_library(vvc SHARED coder.cpp)
target_link_libraries(vvc ${PCL_LIBRARIES} libturbojpeg.so)
