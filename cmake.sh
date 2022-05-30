if [ ! -f ./dll/libturbojpeg.so ] ||  [ ! -f ./dll/libzstd.so ]; then
    source ./external.sh
fi

if [ ! -d ./build/ ]; then
    mkdir build
    cd build
    cmake -DINIT=Y -DCMAKE_BUILD_TYPE=Release ..
else
    cd build
fi
make
cd ../
