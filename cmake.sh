###
 # @Author: ChenRP07
 # @Date: 2022-05-26 17:11:47
 # @LastEditTime: 2022-05-27 10:39:30
 # @LastEditors: ChenRP07
 # @Description: 
### 
if [! -d ./build/ ]; then
    mkdir build
    cd build
    cmake -DINIT=Y -DCMAKE_BUILD_TYPE=Release ..
else
    cd build
fi
make
