#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <emscripten/val.h>

#include "../embind.hpp"
#include "pcd_data_reader.hpp"
#include "pcd_data_writer.hpp"

using namespace emscripten;
using namespace pcl;

template <typename PointT>
void loadPCDData(val const &data, PointCloud<PointT> &cloud) {}

template <typename PointT>
auto savePCDData(const PointCloud<PointT> &cloud, const bool binary_mode = false) {
  const char *data;
  std::size_t data_size = 0;

  if (binary_mode) {
    data = writeBinary<PointT>(cloud, data_size);
  } else {
    data = writeASCII<PointT>(cloud, data_size);
  }

  return val(typed_memory_view(data_size, data));
}

template <typename PointT>
auto savePCDDataBinaryCompressed(const PointCloud<PointT> &cloud) {
  std::size_t data_size = 0;
  const char *data = writeBinaryCompressed(cloud, data_size);

  return val(typed_memory_view(data_size, data));
}

#define BIND_loadPCDData(r, data, PointT) \
  function("loadPCDData" BOOST_PP_STRINGIZE(PointT), &loadPCDData);

#define BIND_savePCDData(r, data, PointT) \
  function("savePCDData" BOOST_PP_STRINGIZE(PointT), &savePCDData<PointT>);

#define BIND_savePCDDataBinaryCompressed(r, data, PointT)            \
  function("savePCDDataBinaryCompressed" BOOST_PP_STRINGIZE(PointT), \
                                                            &savePCDDataBinaryCompressed<PointT>);
// TODO: Node.js
// #define BIND_loadPCDFile(r, data, PointT) \
//   function("loadPCDFile" BOOST_PP_STRINGIZE(PointT), select_overload<int(const std::string &,
//   PointCloud<PointT> &)>(&io::loadPCDFile));

// #define BIND_savePCDFile(r, data, PointT)                                          \
//   function("savePCDFile" BOOST_PP_STRINGIZE(PointT),                                                                     \
//       select_overload<int(const std::string &, const PointCloud<PointT> &, bool)>( \
//           &io::savePCDFile));

// #define BIND_savePCDFileBinaryCompressed(r, data, PointT) \
//   function("savePCDFileBinaryCompressed" BOOST_PP_STRINGIZE(PointT), &io::savePCDFileBinaryCompressed<PointT>);

EMSCRIPTEN_BINDINGS(io) {
  // BOOST_PP_SEQ_FOR_EACH(BIND_loadPCDData, , POINT_TYPES);
  BOOST_PP_SEQ_FOR_EACH(BIND_savePCDData, , POINT_TYPES);
  BOOST_PP_SEQ_FOR_EACH(BIND_savePCDDataBinaryCompressed, , POINT_TYPES);

  // BOOST_PP_SEQ_FOR_EACH(BIND_loadPCDFile, , POINT_TYPES);
  // BOOST_PP_SEQ_FOR_EACH(BIND_savePCDFile, , POINT_TYPES);
  // BOOST_PP_SEQ_FOR_EACH(BIND_savePCDFileBinaryCompressed, , POINT_TYPES);
}