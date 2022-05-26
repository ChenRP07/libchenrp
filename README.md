<!--
 * @Author: ChenRP07
 * @Date: 2022-05-22 16:59:30
 * @LastEditTime: 2022-05-26 17:05:30
 * @LastEditors: ChenRP07
 * @Description: 
-->
# libvvc
## This is an library of MortonCoding, Turbo-jpegCoding and ZstdCoding.
## Clone and Build
### Unix
Make sure CMake version at least 3.19.0

Suppose your project directory is ~/DIR/

First clone this library from GitHub.

`git clone --recurse-submodules -b vvc git@github.com:ChenRP07/libchenrp.git ~/DIR/libvvc/`

Then build it.

`source ./cmake.sh`

Our bash script will automatically build and make the libvvc. 

If you want to use libvvc in your project ~/DIR/, you may run an install srcipt 

`source ./install.sh ~/DIR`

Make sure you project has these two directory ~/DIR/include and ~/DIR/dll

Your CMakeLists.txt should contain

`target_link_libraries(YOURNAME libturbojpeg.so.0 libzstd.so libvvc.so)`

or any code equal to it. 