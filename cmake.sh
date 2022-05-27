source ./external.sh
if [ ! -d ./build/ ]; then
    mkdir build
    cd build
    cmake -DINIT=Y -DCMAKE_BUILD_TYPE=Release ..
else
    cd build
fi
make
cd ../
