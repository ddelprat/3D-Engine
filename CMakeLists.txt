cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP)  # Use OpenMP if available

if(OpenMP_CXX_FOUND)
    message(STATUS "Using OpenMP with LLVM Clang")

    if(APPLE)
        # Get Homebrew prefix for LLVM and libomp
        execute_process(COMMAND brew --prefix llvm OUTPUT_VARIABLE LLVM_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)
        execute_process(COMMAND brew --prefix libomp OUTPUT_VARIABLE LIBOMP_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE)

        # Use LLVM Clang with OpenMP
        set(OpenMP_CXX_FLAGS "-fopenmp -I${LIBOMP_PREFIX}/include")
        set(OpenMP_CXX_LIBRARIES "${LIBOMP_PREFIX}/lib/libomp.dylib")

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_LIBRARIES}")
    else()
        add_compile_options(-fopenmp)
        add_link_options(-fopenmp)
    endif()
else()
    message(WARNING "OpenMP not found! Your program will run in single-threaded mode.")
endif()

add_executable(main main.cpp)
target_link_libraries(main OpenMP::OpenMP_CXX)




