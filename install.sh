###
 # @Author: ChenRP07
 # @Date: 2022-05-26 17:13:29
 # @LastEditTime: 2022-05-27 10:32:44
 # @LastEditors: ChenRP07
 # @Description: install
### 

echo "libcluster will be installed to $1"
cp ./include/* $1/include/
cp ./release/* $1/dll/
