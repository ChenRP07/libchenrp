add_library(cluster SHARED Patch.cpp SimplePatch.cpp)
target_link_libraries(cluster ${PCL_LIBRARIES})

add_library(io SHARED IO.cpp)
target_link_libraries(io ${PCL_LIBRARIES})