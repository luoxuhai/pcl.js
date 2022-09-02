#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/bind.h>

using namespace pcl;

PointCloud<PointXYZ>::Ptr loadPCDFile(const std::string &file_name)
{
  PointCloud<PointXYZ>::Ptr cloud_ptr(new PointCloud<PointXYZ>);

  int flag = io::loadPCDFile(file_name, *cloud_ptr);

  return cloud_ptr;
}

using namespace emscripten;

EMSCRIPTEN_BINDINGS(io)
{
  function("io.loadPCDFile", &loadPCDFile);
  function("io.savePCDFile", select_overload<int(const std::string &, const PointCloud<PointXYZ> &, bool)>(&io::savePCDFile));
  function("io.savePCDFileBinaryCompressed", &io::savePCDFileBinaryCompressed<PointXYZ>);
}