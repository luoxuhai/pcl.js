#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/bind.h>

using namespace emscripten;

pcl::PointCloud<pcl::PointXYZ> loadPCDFile(const std::string &file_name)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> &cloud = *cloud_ptr;

  int flag = pcl::io::loadPCDFile(file_name, cloud);

  return cloud;
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
  class_<pcl::PointCloud<pcl::PointXYZ>>("PointCloudXYZ")
      .constructor<>()
      .property("width", &pcl::PointCloud<pcl::PointXYZ>::width)
      .property("height", &pcl::PointCloud<pcl::PointXYZ>::height)
      .property("points", &pcl::PointCloud<pcl::PointXYZ>::points)
      .function("isOrganized", &pcl::PointCloud<pcl::PointXYZ>::isOrganized);
  register_vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>("Points");
}