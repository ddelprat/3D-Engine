# 3D-Engine

This repository is a 3D engine in C++ following this tutorial :
https://perso.liris.cnrs.fr/nicolas.bonneel/teaching.html


Here is an example of a result :

![img.png](img.png)

## Method to compile and run on MacOS :

Create a `build_run.sh` file :

    touch build_run.sh
    chmod +x build_run.sh


And copy the following code (if using llvm downloaded from Homebrew) :

    #!/bin/bash
    
    # Stop script on any error
    set -e
    
    # Clean previous build
    rm -rf build
    mkdir build
    cd build
    
    # Get LLVM and OpenMP paths
    LLVM_PREFIX=$(brew --prefix llvm)
    LIBOMP_PREFIX=$(brew --prefix libomp)
    
    # Run CMake with LLVM Clang
    cmake -DCMAKE_C_COMPILER="$LLVM_PREFIX/bin/clang" \
    -DCMAKE_CXX_COMPILER="$LLVM_PREFIX/bin/clang++" \
    ..
    
    # Compile using all available CPU cores
    make -j$(sysctl -n hw.logicalcpu)
    
    ./main