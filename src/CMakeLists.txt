add_library(octree SHARED operation.cpp GOF.cpp octree.cpp)
target_link_libraries(octree ${PCL_LIBRARIES})