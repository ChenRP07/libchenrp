add_library(octree SHARED operation.cpp GOF.cpp octree.cpp deoctree.cpp)
target_link_libraries(octree ${PCL_LIBRARIES})