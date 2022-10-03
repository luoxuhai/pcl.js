#include <pcl/keypoints/iss_3d.h>

#include "embind.hpp"

using namespace pcl;
using namespace emscripten;

#define BIND_Keypoint(r, data, PointT)                                                          \
  class_<pcl::Keypoint<PointT, PointT>, base<pcl::PCLBase<PointT>>>(                            \
      "Keypoint" BOOST_PP_STRINGIZE(PointT))                                                    \
          .function("setSearchMethod", &pcl::Keypoint<PointT, PointT>::setSearchMethod)         \
          .function("getSearchMethod", &pcl::Keypoint<PointT, PointT>::getSearchMethod)         \
          .function("getSearchParameter", &pcl::Keypoint<PointT, PointT>::getSearchParameter)   \
          .function("setKSearch", &pcl::Keypoint<PointT, PointT>::setKSearch)                   \
          .function("getKSearch", &pcl::Keypoint<PointT, PointT>::getKSearch)                   \
          .function("setRadiusSearch", &pcl::Keypoint<PointT, PointT>::setRadiusSearch)         \
          .function("getRadiusSearch", &pcl::Keypoint<PointT, PointT>::getRadiusSearch)         \
          .function("getKeypointsIndices", &pcl::Keypoint<PointT, PointT>::getKeypointsIndices) \
          .function("compute", &pcl::Keypoint<PointT, PointT>::compute);

#define BIND_ISSKeypoint3D(r, data, PointT)                                                    \
  class_<pcl::ISSKeypoint3D<PointT, PointT>, base<pcl::Keypoint<PointT, PointT>>>(             \
      "ISSKeypoint3D" BOOST_PP_STRINGIZE(PointT))                                              \
          .constructor<double>()                                                               \
          .function("setSalientRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setSalientRadius) \
          .function("setNonMaxRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setNonMaxRadius)   \
          .function("setNormalRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setNormalRadius)   \
          .function("setBorderRadius", &pcl::ISSKeypoint3D<PointT, PointT>::setBorderRadius)   \
          .function("setMinNeighbors", &pcl::ISSKeypoint3D<PointT, PointT>::setMinNeighbors)   \
          .function("setThreshold21", &pcl::ISSKeypoint3D<PointT, PointT>::setThreshold21)     \
          .function("setThreshold32", &pcl::ISSKeypoint3D<PointT, PointT>::setThreshold32)     \
          .function("setAngleThreshold", &pcl::ISSKeypoint3D<PointT, PointT>::setAngleThreshold);

EMSCRIPTEN_BINDINGS(keypoints) {
  BOOST_PP_SEQ_FOR_EACH(BIND_Keypoint, , XYZ_POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_ISSKeypoint3D, , XYZ_POINT_TYPES);
}