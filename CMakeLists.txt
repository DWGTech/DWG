cmake_minimum_required(VERSION 3.1)

project(DWG_CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

### Compiler options
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 ) # Emit a compile flags file to support completion engines 

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using Clang (linux or apple) or GCC
  message("Using clang/gcc compiler flags")
  SET(BASE_CXX_FLAGS "-std=c++17 -Wall -Wextra -g3")
  SET(DISABLED_WARNINGS " -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-deprecated-declarations -Wno-missing-braces")
  SET(TRACE_INCLUDES " -H -Wno-error=unused-command-line-argument")

  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    message("Setting clang-specific options")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -ferror-limit=5 -fcolor-diagnostics")
    SET(CMAKE_CXX_FLAGS_DEBUG          "-fsanitize=address -fno-limit-debug-info")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -fmax-errors=5")
    message("Setting gcc-specific options")
    SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} -Wno-maybe-uninitialized -Wno-format-zero-length -Wno-unused-but-set-parameter -Wno-unused-but-set-variable")
  endif()


  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TRACE_INCLUDES}") # uncomment if you need to track down where something is getting included from
  SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -g3")
  SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELEASE        "-march=native -O3 -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
  message("Using Visual Studio compiler flags")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # parallel build
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4267\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4244\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4305\"")  # ignore truncation on initialization
  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
#  add_compile_options(/arch:AVX2)

  add_definitions(/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions(-DNOMINMAX)
  add_definitions(-D_USE_MATH_DEFINES)

else()
  # unrecognized
  message( FATAL_ERROR "Unrecognized compiler [${CMAKE_CXX_COMPILER_ID}]" )
endif()

enable_language(CUDA)
message(STATUS "CUDA enabled")
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_RELEASE "")
set(CMAKE_CUDA_FLAGS_DEBUG "")
set( CMAKE_CUDA_STANDARD 14 )
set( CMAKE_CUDA_STANDARD_REQUIRED ON)
##set( CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS}  -g -Xcudafe \"--display_error_number --diag_suppress=3057 --diag_suppress=3058 --diag_suppress=3059 --diag_suppress=3060\" -lineinfo")

###set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -Xcompiler=\"/MT\"" )
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --use_fast_math")
set(CMAKE_CUDA_ARCHITECTURES OFF)

include_directories ( ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

### set(BOOST_ROOT "E:/boost_1_71_0")
### set(BOOST_INCLUDEDIR "E:/boost_1_71_0/boost")
### set(BOOST_LIBRARYDIR "E:/boost_1_71_0/lib64-msvc-14.2")

### Policy settings
cmake_policy(SET CMP0002 NEW)   # don't implicitly dereference inside if()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


set(LIBIGL_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/deps/libigl/include)

### libIGL options: choose your dependencies (by default everything is OFF, in this example we need the viewer) ###
find_package(CGAL QUIET COMPONENTS Core)
option(LIBIGL_WITH_CGAL             "Use CGAL"           "${CGAL_FOUND}")
option(LIBIGL_WITH_COMISO           "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK             "Use CORK"           OFF)
option(LIBIGL_WITH_EMBREE           "Use Embree"         OFF)
option(LIBIGL_WITH_LIM              "Use LIM"            OFF)
#find_package(MATLAB QUIET)
option(LIBIGL_WITH_MATLAB           "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK            "Use MOSEK"          "${MOSEK_FOUND}")
option(LIBIGL_WITH_OPENGL           "Use OpenGL"         OFF)
option(LIBIGL_WITH_OPENGL_GLFW      "Use OpenGL GLFW"    OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use OpenGL_GLFW_IMGUI" OFF)
option(LIBIGL_WITH_PNG              "Use PNG"            OFF)
option(LIBIGL_WITH_TETGEN           "Use Tetgen"         OFF)
option(LIBIGL_WITH_TRIANGLE         "Use Triangle"       OFF)
option(LIBIGL_WITH_XML              "Use XML"            OFF)
### End   to be tested ----





message(STATUS "libigl ended")



find_package(LIBIGL REQUIRED QUIET)


# add openMP
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "found openmp") 
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(FATAL_ERROR "openmp not found!")
endif()

# Add your project files
file(GLOB SRCFILES 
	src/*.cpp
	src/*.cu
    "include/*.h"
    "src/*.cu"
)

find_package(PCL REQUIRED)
include_directories("/usr/include/eigen3")
include_directories("/usr/include/pcl-1.10")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} ${SRCFILES} "include/utils.h")

#target_compile_options(${PROJECT_NAME} PUBLIC  -std=c++17)
#target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)


target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)


target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/libigl/include")
 target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/cudaKDTree")
 target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
  target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/geometrycentral")


### libIGL options: decide if you want to use the functionalities that depends on cgal
#[[
if(LIBIGL_WITH_CGAL) # Do not remove or move this block, cgal strange build system fails without it
set (CGAL_DIR /root/autodl-tmp/CGAL-5.5.1)
  find_package(CGAL REQUIRED COMPONENTS Core)
  set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "CGAL's CMAKE Setup is super annoying ")
  #include(${CGAL_USE_FILE})
  target_link_libraries(${PROJECT_NAME} CGAL::CGAL)
endif()
]]

target_link_libraries(${PROJECT_NAME} igl::core igl::cgal )
target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
