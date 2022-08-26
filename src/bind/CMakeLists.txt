cmake_minimum_required(VERSION 3.22)

# Base
set(PROJECT_NAME pcl-core)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

set(PCL_INCLUDE ${PCL_ROOT}/build/include/pcl-1.12)
set(BOOST_INCLUDE ${PCL_ROOT}/wasm/deps/boost/build/include)
set(EIGEN_INCLUDE ${PCL_ROOT}/wasm/deps/eigen)
set(FLANN_INCLUDE ${PCL_ROOT}/wasm/deps/flann/build/include)

set(PCL_LIB ${PCL_ROOT}/build/lib)
set(BOOST_LIB ${PCL_ROOT}/wasm/deps/boost/build/lib)
set(FLANN_LIB ${PCL_ROOT}/wasm/deps/flann/build/lib)

# Link library
include_directories(${PCL_INCLUDE} ${BOOST_INCLUDE} ${EIGEN_INCLUDE} ${FLANN_INCLUDE})
link_directories(${PCL_LIB} ${BOOST_LIB} ${FLANN_LIB})

set(PCL_LINK_LIBS pcl_common pcl_io pcl_features pcl_filters pcl_kdtree pcl_stereo pcl_keypoints pcl_octree pcl_search pcl_segmentation pcl_surface pcl_tracking pcl_ml)
set(SRCS main.cpp io.cpp filters.cpp kdtree.cpp octree.cpp keypoints.cpp registration.cpp point_types.cpp)

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${PCL_LINK_LIBS} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)

option(CMAKE_BUILD_TYPE "This is a option for build type" "Debug")

set(COMMON_LINK_FLAGS "-lembind -sSTRICT=1 -sALLOW_MEMORY_GROWTH=1 -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=PCLCore -sENVIRONMENT=web,webview,worker -sINCOMING_MODULE_JS_API=fetchSettings,locateFile -sEXPORTED_RUNTIME_METHODS=FS")

set(EM_LINK_FLAGS "${COMMON_LINK_FLAGS} -g")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  set(EM_LINK_FLAGS "${COMMON_LINK_FLAGS} -O3")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${EM_LINK_FLAGS})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})