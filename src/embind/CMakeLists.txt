cmake_minimum_required(VERSION 3.22)

# Base
set(PROJECT_NAME pcl-core)
set(CMAKE_INSTALL_BINDIR ../../dist)
project(${PROJECT_NAME})
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

set(PCL_LINK_LIBS pcl_common pcl_io pcl_features pcl_filters pcl_kdtree pcl_stereo pcl_keypoints pcl_octree pcl_search pcl_segmentation pcl_surface pcl_tracking pcl_ml)
set(SRCS main.cpp io.cpp filters.cpp kdtree.cpp octree.cpp keypoints.cpp registration.cpp point_types.cpp)

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${PCL_LINK_LIBS} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-g -lembind -s STRICT=1 -s EXPORTED_RUNTIME_METHODS=FS -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_ES6=1")

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})