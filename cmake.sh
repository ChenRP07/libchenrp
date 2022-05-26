source ./external.sh
if [ ! -d ./build ]; then
	mkdir build
fi
cd build
cmake -DINIT=Y -DCMAKE_BUILD_TYPE=Release ..
make
cd ../
