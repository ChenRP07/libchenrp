add_library(vvc SHARED coder.cpp)
target_link_libraries(vvc libturbojpeg.so.0 libzstd.so)
