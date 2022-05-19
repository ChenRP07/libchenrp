add_library(registration SHARED regist_base.cpp operation.cpp ICP.cpp NICP.cpp ParallelICP.cpp)
target_link_libraries(registration ${PCL_LIBRARIES})