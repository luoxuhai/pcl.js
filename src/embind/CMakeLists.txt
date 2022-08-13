cmake_minimum_required(VERSION 3.22)

# Base
set(MODULE_NAME pcljs-io)
project(${MODULE_NAME})
set(CMAKE_CXX_STANDARD 14)

# Directory
set(ROOT /Users/luoxuhai/Desktop/repo/pcl.js/core/pcl)

set(PCL_INCLUDE ${ROOT}/build/include/pcl-1.12)
set(BOOST_INCLUDE ${ROOT}/wasm/deps/boost/build/include)
set(EIGEN_INCLUDE ${ROOT}/wasm/deps/eigen)
set(FLANN_INCLUDE ${ROOT}/wasm/deps/flann/build/include)

set(PCL_LIB ${ROOT}/build/lib)
set(BOOST_LIB ${ROOT}/wasm/deps/boost/build/lib)
set(FLANN_LIB ${ROOT}/wasm/deps/flann/build/lib)

# wasm
set(CMAKE_EXECUTABLE_SUFFIX ".html")

# Link library
include_directories(${PCL_INCLUDE} ${BOOST_INCLUDE} ${EIGEN_INCLUDE} ${FLANN_INCLUDE})
link_directories(${PCL_LIB} ${BOOST_LIB} ${FLANN_LIB})

add_executable(${MODULE_NAME} ./test.cpp)
target_link_libraries(${MODULE_NAME} pcl_common pcl_io flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)
set_target_properties(${MODULE_NAME} PROPERTIES LINK_FLAGS "-G -lembind -s EXPORTED_RUNTIME_METHODS=FS -s ALLOW_MEMORY_GROWTH=1 -fno-builtin")