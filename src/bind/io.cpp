#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/bind.h>

using namespace emscripten;

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDFile(const std::string &file_name)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  int flag = pcl::io::loadPCDFile(file_name, *cloud_ptr);

  return cloud_ptr;
}

int savePCDFile(const std::string &file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud, bool binary_mode = false)
{
  return pcl::io::savePCDFile(file_name, cloud, binary_mode);
}

EMSCRIPTEN_BINDINGS(io)
{
  function("io.loadPCDFile", &loadPCDFile);
  function("io.savePCDFile", &savePCDFile);
  function("io.savePCDFileBinaryCompressed", &pcl::io::savePCDFileBinaryCompressed<pcl::PointXYZ>);

  class_<pcl::PointCloud<pcl::PointXYZ>>("PointCloudXYZ")
      .constructor<>()
      .property("width", &pcl::PointCloud<pcl::PointXYZ>::width)
      .property("height", &pcl::PointCloud<pcl::PointXYZ>::height)
      .property("points", &pcl::PointCloud<pcl::PointXYZ>::points)
      .property("is_dense", &pcl::PointCloud<pcl::PointXYZ>::is_dense)
      .function("isOrganized", &pcl::PointCloud<pcl::PointXYZ>::isOrganized)
      .smart_ptr<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>("PointCloudXYZ")
      .smart_ptr<pcl::PointCloud<pcl::PointXYZ>::Ptr>("PointCloudXYZ");

  register_vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>("Points");
}