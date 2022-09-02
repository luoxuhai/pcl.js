#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/bind.h>
#include "embind.cpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(common)
{
  class_<pcl::PointCloud<pcl::PointXYZ>>("PointCloudXYZ")
      .constructor<>()
      .property("width", &pcl::PointCloud<pcl::PointXYZ>::width)
      .property("height", &pcl::PointCloud<pcl::PointXYZ>::height)
      .property("points", &pcl::PointCloud<pcl::PointXYZ>::points)
      .property("is_dense", &pcl::PointCloud<pcl::PointXYZ>::is_dense)
      .function("isOrganized", &pcl::PointCloud<pcl::PointXYZ>::isOrganized)
      .smart_ptr<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>("PointCloudXYZ")
      .smart_ptr<pcl::PointCloud<pcl::PointXYZ>::Ptr>("PointCloudXYZ");

  register_vector_plus<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>("Points");
}