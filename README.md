<!--
 * @Author: ChenRP07
 * @Date: 2022-05-26 17:11:47
 * @LastEditTime: 2022-05-27 10:14:51
 * @LastEditors: ChenRP07
 * @Description: readme
-->
# liboctree
## This is an library of Octree, Point and Cloud operation, Group Of Frames.
## Clone and Build
### Unix
Make sure CMake version at least 3.19.0

Suppose your project directory is ~/DIR/

First clone this library from GitHub.

`git clone -b octree git@github.com:ChenRP07/libchenrp.git ~/DIR/libcluster/`

Then build it.

`source ./cmake.sh`

Our bash script will automatically build and make the liboctree.

If you want to use liboctree in your project ~/DIR/, you may run an install srcipt.

`source ./install.sh ~/DIR`

Make sure you project has these two directory ~/DIR/include and ~/DIR/dll

Your CMakeLists.txt should contain

`target_link_libraries(YOURNAME libcluster.so)`

or any code equal to it.

@copyright ChenRP07