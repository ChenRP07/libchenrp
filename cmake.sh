###
 # @Author: ChenRP07
 # @Date: 2022-05-31 16:56:32
 # @LastEditTime: 2022-05-31 16:57:15
 # @LastEditors: ChenRP07
 # @Description: 
### 

source ./extern.sh

if [ ! -d ./build/ ]; then
    mkdir build
    cd build
    cmake -DINIT=Y -DCMAKE_BUILD_TYPE=Release ..
else
    cd build
fi
make
cd ../