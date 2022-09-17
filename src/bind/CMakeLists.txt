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

file(GLOB PCL_LINK_LIBS ${PCL_LIB}/*.a)
file(GLOB SRCS *.cpp)

foreach(SRC IN LISTS SRCS)
    message(STATUS ${SRC})
endforeach(SRC)

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${PCL_LINK_LIBS} flann_cpp_s boost_filesystem boost_regex boost_system boost_date_time boost_iostreams)

option(CMAKE_BUILD_TYPE "This is a option for build type" "Debug")

set(INCOMING_MODULE_JS_API "fetchSettings,locateFile,wasmBinary")
set(COMMON_LINK_FLAGS "-lembind -sSTRICT=1 -sALLOW_MEMORY_GROWTH=1 -sMODULARIZE=1 -sEXPORT_ES6=1 -sUSE_ES6_IMPORT_META=0 -sEXPORT_NAME=PCLCore -sENVIRONMENT=web,webview,worker -sINCOMING_MODULE_JS_API=${INCOMING_MODULE_JS_API} -sEXPORTED_RUNTIME_METHODS=FS --no-entry")

set(EM_LINK_FLAGS "${COMMON_LINK_FLAGS} -g")
if (${CMAKE_BUILD_TYPE} STREQUAL "Release")
  set(EM_LINK_FLAGS "${COMMON_LINK_FLAGS} -O3")
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS ${EM_LINK_FLAGS})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})