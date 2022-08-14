#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/bind.h>

using namespace emscripten;

int loadPCDFile(const std::string &file_name, pcl::PCLPointCloud2 &cloud)
{
  return pcl::io::loadPCDFile(file_name, cloud);
}

int savePCDFile(const std::string &file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud, bool binary_mode = false)
{
  return pcl::io::savePCDFile(file_name, cloud, binary_mode);
}

int savePCDFileBinaryCompressed(const std::string &file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  return pcl::io::savePCDFileBinaryCompressed(file_name, cloud);
}

EMSCRIPTEN_BINDINGS(io)
{
  function("io.loadPCDFile", &loadPCDFile);
  function("io.savePCDFile", &savePCDFile);
  function("io.savePCDFileBinaryCompressed", &savePCDFileBinaryCompressed);
}