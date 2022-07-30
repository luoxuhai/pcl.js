CC = emcc

EMSDK_DIR=/Users/luoxuhai/Desktop/WebAssembly/emsdk

OUT_DIR = .

EMFLAGS = \
	-s MODULARIZE=1 \
	-s EXPORT_ES6=1 \
	-s USE_ES6_IMPORT_META=0 \
	-s EXPORT_NAME=PCL \
	-s ALLOW_MEMORY_GROWTH=1 \
	-O3 \
	-lembind
 
ROOT = /Users/luoxuhai/Desktop/repo/pcl.js/core/pcl

PCL_INCLUDE = $(ROOT)/build/include/pcl-1.12
BOOST_INCLUDE = $(ROOT)/wasm/deps/boost/build/include
EIGEN_INCLUDE = $(ROOT)/wasm/deps/eigen/build/include
PCL_LIB = $(ROOT)/build/lib
BOOST_LIB = $(ROOT)/wasm/deps/boost/build/lib
EIGEN_LIB = $(ROOT)/wasm/deps/eigen/build/lib

build-obj:
	$(CC) io.cpp -c -o io.o -I ${INCLUDE} -I $(BOOST_INCLUDE) -I $(EIGEN_INCLUDE)

# build-wasm:
#  $(CC) io.o $(EMFLAGS) -L ${PCL_LIB} -L $(BOOST_LIB) -L $(EIGEN_LIB) -lpcl_common -lpcl_kdtree -lpcl_io -o io.html

# // /Users/luoxuhai/Desktop/repo/pcl.js/src/embind