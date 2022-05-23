cp 3rd/libjpeg-turbo/turbojpeg.h ./include/
mkdir dll
cd 3rd/libjpeg-turbo/
mkdir build
cd build
cmake -G"Unix Makefiles" ..
make
cp 3rd/libjpeg-turbo/build/libturbojpeg.so.0 ./dll/
