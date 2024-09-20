# Diffusing Winding Gradients (DWG): A Parallel and Scalable Method for 3D Reconstruction from Unoriented Point Clouds
## Building
Please install [CMake](https://cmake.org) 
### Dependencies
  * CGAL-5.6.1
  * PCL
  * Eigen
  * CUDA (We tested on CUDA 11.1 and 11.3)
### Windows 10
    mkdir build && cd build
    cmake .. -G"Visual Studio 16 2019"
    cmake --build . --config Release
### Linux
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

    
  

