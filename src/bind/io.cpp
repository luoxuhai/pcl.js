#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/val.h>

#include "embind.hpp"

using namespace emscripten;
using namespace pcl;

#define BIND_loadPCDFile(r, data, PointT) \
  function("loadPCDFile" BOOST_PP_STRINGIZE(PointT), select_overload<int(const std::string &, PointCloud<PointT> &)>(&io::loadPCDFile));


PointCloud<pcl::PointXYZ>::Ptr BIND_loadPointByArrayXYZ(val array) {
    unsigned int length = array["length"].as<float>();
    
    PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointXYZ point;
    for(unsigned int i = 0; i < length / 3; ++i) {
      point.x = array[i * 3].as<float>();
      point.y = array[i * 3 + 1].as<float>();
      point.z = array[i * 3 + 2].as<float>();
      cloud->push_back(point);
    }
    return cloud;
} 


#define BIND_savePCDFile(r, data, PointT)                                          \
  function("savePCDFile" BOOST_PP_STRINGIZE(PointT),                                                                     \
      select_overload<int(const std::string &, const PointCloud<PointT> &, bool)>( \
          &io::savePCDFile));

#define BIND_savePCDFileBinaryCompressed(r, data, PointT) \
  function("savePCDFileBinaryCompressed" BOOST_PP_STRINGIZE(PointT), &io::savePCDFileBinaryCompressed<PointT>);

EMSCRIPTEN_BINDINGS(io) {
  BOOST_PP_SEQ_FOR_EACH(BIND_loadPCDFile, , POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_savePCDFile, , POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_savePCDFileBinaryCompressed, , POINT_TYPES);

  function("loadPointByArrayXYZ",&BIND_loadPointByArrayXYZ);
}