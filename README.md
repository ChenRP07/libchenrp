# libchenrp
This is a C++ project template.

Project template consists of 

    |--MyProject
    |   |--include
    |   |   |--MyProject.h
    |   |   |--...
    |   |--src
    |   |   |--MyProject.cpp
    |   |   |--CMakeLists.txt
    |   |   |--...
    |   |--build
    |   |   |--compile_commands.json
    |   |   |--...
    |   |--dll
    |   |   |--lib3rd.so
    |   |   |--...
    |   |--3rd
    |   |   |--lib3rd
    |   |   |   |--...
    |   |--release
    |   |   |--libMyProject.so
    |   |   |--...
    |   |--debug
    |   |   |--libMyProject.so
    |   |   |--...
    |   |--test
    |   |   |--CMakeLists.txt
    |   |   |--test.cpp
    |   |   |--...
    |   |--CMakeLists.txt
    |   |--cmake.sh
    |   |--extern.sh
    |   |--.gitignore
    |   |--README.md

Dir *include* contains all C++ head files, and dir *src* contains all C++ source files, and *src/CMakeLists.txt* will compile those source files to a dynamic library. Dir *3rd* contains all submodules, *dll* contains all extern libraries, *release* contains the release mode of the library this project generate, also *debug* contains the debug mode. Dir *test* contains all test file during the coding, *test/CMakeLists.txt* will help you build it.

*CMakeLists.txt* will build the whole project, *extern.sh* will build the 3rd libraries and *cmake.sh* will automaticlly do the above two job.