#include <pcl/registration/icp.h>

#include "embind.hpp"

using namespace pcl;
using namespace emscripten;

#define BIND_Registration(r, data, PointT)                                            \
  class_<pcl::Registration<PointT, PointT>>(                                          \
      "Registration" BOOST_PP_STRINGIZE(PointT))                                      \
          .function("hasConverged", &pcl::Registration<PointT, PointT>::hasConverged) \
          .function("getFinalTransformation",                                         \
                    &pcl::Registration<PointT, PointT>::getFinalTransformation)       \
          .function("getFitnessScore", &getFitnessScore<PointT>)                      \
          .function("align", &align<PointT>);

#define BIND_IterativeClosestPoint(r, data, PointT)                                                \
  class_<pcl::IterativeClosestPoint<PointT, PointT>, base<pcl::Registration<PointT, PointT>>>(     \
      "IterativeClosestPoint" BOOST_PP_STRINGIZE(PointT))                                          \
          .constructor<>()                                                                         \
          .function("setInputSource", &pcl::IterativeClosestPoint<PointT, PointT>::setInputSource) \
          .function("setInputTarget", &pcl::IterativeClosestPoint<PointT, PointT>::setInputTarget) \
          .function("setUseReciprocalCorrespondences",                                             \
                    &pcl::IterativeClosestPoint<PointT, PointT>::setUseReciprocalCorrespondences)  \
          .function("getUseReciprocalCorrespondences",                                             \
                    &pcl::IterativeClosestPoint<PointT, PointT>::getUseReciprocalCorrespondences);

template <typename PointT>
auto align(pcl::Registration<PointT, PointT> &registration,
           typename pcl::PointCloud<PointT>::Ptr &output) {
  if (output == nullptr) {
    pcl::PointCloud<PointT> cloud;
    registration.align(cloud);
    return cloud.makeShared();
  } else {
    registration.align(*output);
    return output;
  }
}

template <typename PointT>
double getFitnessScore(pcl::Registration<PointT, PointT> &registration) {
  return registration.getFitnessScore();
}

EMSCRIPTEN_BINDINGS(registration) {
  // Bind Registration
  BOOST_PP_SEQ_FOR_EACH(BIND_Registration, , XYZ_POINT_TYPES);

  // Bind IterativeClosestPoint
  BOOST_PP_SEQ_FOR_EACH(BIND_IterativeClosestPoint, , XYZ_POINT_TYPES);
}