#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_config.h>
#include <emscripten/bind.h>
#include "embind.cpp"

const std::string VERSION = PCL_VERSION_PRETTY;

#define BIND_POINT_CLOUD(PointT)                                   \
  class_<PointCloud<PointT>>("PointCloud" #PointT)                 \
      .constructor<>()                                             \
      .property("width", &PointCloud<PointT>::width)               \
      .property("height", &PointCloud<PointT>::height)             \
      .property("points", &PointCloud<PointT>::points)             \
      .property("is_dense", &PointCloud<PointT>::is_dense)         \
      .function("isOrganized", &PointCloud<PointT>::isOrganized)   \
      .function("clear", &PointCloud<PointT>::clear)               \
      .function("makeShared", &PointCloud<PointT>::makeShared)     \
      .smart_ptr<PointCloud<PointT>::ConstPtr>("ConstPtr" #PointT) \
      .smart_ptr<PointCloud<PointT>::Ptr>("Ptr" #PointT);

#define BIND_POINTS(PointT) register_vector_plus<PointT, Eigen::aligned_allocator<PointT>>("Points" #PointT);

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(common)
{
  constant("PCL_VERSION", VERSION);

  BIND_POINT_CLOUD(PointXYZ);
  BIND_POINT_CLOUD(PointXYZI);
  BIND_POINT_CLOUD(PointXYZRGB);
  BIND_POINT_CLOUD(PointXYZRGBA);
  BIND_POINT_CLOUD(Normal);
  BIND_POINT_CLOUD(PointNormal);

  BIND_POINTS(PointXYZ);
  BIND_POINTS(PointXYZI);
  BIND_POINTS(PointXYZRGB);
  BIND_POINTS(PointXYZRGBA);
  BIND_POINTS(Normal);
  BIND_POINTS(PointNormal);
}