<!--
 * @Author: ChenRP07
 * @Date: 2022-05-26 17:11:47
 * @LastEditTime: 2022-05-27 09:34:39
 * @LastEditors: ChenRP07
 * @Description: readme
-->
# libregistration
## This is an library of ICP, GICP, NICP.
## Clone and Build
### Unix
Make sure CMake version at least 3.19.0

Suppose your project directory is ~/DIR/

First clone this library from GitHub.

`git clone -b registration git@github.com:ChenRP07/libchenrp.git ~/DIR/libcluster/`

Then build it.

`source ./cmake.sh`

Our bash script will automatically build and make the libregistration.

If you want to use libregistration in your project ~/DIR/, you may run an install srcipt.

`source ./install.sh ~/DIR`

Make sure you project has these two directory ~/DIR/include and ~/DIR/dll

Your CMakeLists.txt should contain

`target_link_libraries(YOURNAME libcluster.so)`

or any code equal to it.

@copyright ChenRP07