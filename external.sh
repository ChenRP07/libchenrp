cp 3rd/libjpeg-turbo/turbojpeg.h ./include/
if [ ! -d ./dll ]; then
    echo create dictory dll
    mkdir dll
fi
cd 3rd/libjpeg-turbo/
if [ ! -d ./build ]; then
    echo create dictory build
    mkdir build
fi
cd build
cmake -G"Unix Makefiles" -DCMAKE_ASM_NASM_COMPILER=/usr/bin/nasm ..
make -j
cd ../../../
cp ./3rd/libjpeg-turbo/build/libturbojpeg.so.0 ./dll/
cp ./3rd/libjpeg-turbo/build/libturbojpeg.so ./dll/
cd 3rd/zstd/
make -j
cd ../../
cp ./3rd/zstd/lib/libzstd.so ./dll/
cp ./3rd/zstd/lib/zstd.h ./include/
rm -rf ./3rd/libjpeg-turbo/build