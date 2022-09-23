#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <emscripten/bind.h>

#define BIND_KEYPOINT(PointT)                                                               \
  class_<pcl::Keypoint<PointT, PointT>, base<pcl::PCLBase<PointT>>>("Keypoint" #PointT)     \
      .function("setSearchMethod", &pcl::Keypoint<PointT, PointT>::setSearchMethod)         \
      .function("getSearchMethod", &pcl::Keypoint<PointT, PointT>::getSearchMethod)         \
      .function("getSearchParameter", &pcl::Keypoint<PointT, PointT>::getSearchParameter)   \
      .function("setKSearch", &pcl::Keypoint<PointT, PointT>::setKSearch)                   \
      .function("getKSearch", &pcl::Keypoint<PointT, PointT>::getKSearch)                   \
      .function("setRadiusSearch", &pcl::Keypoint<PointT, PointT>::setRadiusSearch)         \
      .function("getRadiusSearch", &pcl::Keypoint<PointT, PointT>::getRadiusSearch)         \
      .function("getKeypointsIndices", &pcl::Keypoint<PointT, PointT>::getKeypointsIndices) \
      .function("compute", &pcl::Keypoint<PointT, PointT>::compute);

#define BIND_ISS_3D(PointT)                                                                        \
  class_<pcl::ISSKeypoint3D<PointT, PointT>, base<pcl::Keypoint<PointT, PointT>>>("ISSKeypoint3D" #PointT) \
      .constructor<double>()                                                                       \
      .function("setSalientRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setSalientRadius)         \
      .function("setNonMaxRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setNonMaxRadius)           \
      .function("setNormalRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setNormalRadius)           \
      .function("setBorderRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setBorderRadius)           \
      .function("setMinNeighbors", &pcl::ISSKeypoint3D<PointT, PointT>::setMinNeighbors)           \
      .function("setThreshold21", &pcl::ISSKeypoint3D<PointT, PointT>::setThreshold21)             \
      .function("setThreshold32", &pcl::ISSKeypoint3D<PointT, PointT>::setThreshold32)             \
      .function("setAngleThreshold", &pcl::ISSKeypoint3D<PointT, PointT>::setAngleThreshold);

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(keypoints)
{

  BIND_KEYPOINT(PointXYZ);
  BIND_KEYPOINT(PointXYZI);
  BIND_KEYPOINT(PointXYZRGB);
  BIND_KEYPOINT(PointXYZRGBA);
  BIND_KEYPOINT(PointNormal);

  BIND_ISS_3D(PointXYZ);
  BIND_ISS_3D(PointXYZI);
  BIND_ISS_3D(PointXYZRGB);
  BIND_ISS_3D(PointXYZRGBA);
  BIND_ISS_3D(PointNormal);
}