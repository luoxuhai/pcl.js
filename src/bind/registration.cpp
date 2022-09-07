#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <emscripten/bind.h>

// define Registration
#define BIND_REGISTRATION(PointT)                                                                     \
  class_<pcl::Registration<PointT, PointT>>("Registration" #PointT)                                   \
      .function("hasConverged", &pcl::Registration<PointT, PointT>::hasConverged)                     \
      .function("getFinalTransformation", &pcl::Registration<PointT, PointT>::getFinalTransformation) \
      .function("getFitnessScore", &getFitnessScore<PointT>)                                          \
      .function("align", &align<PointT>);

// define IterativeClosestPoint
#define BIND_ICP(PointT)                                                                                                         \
  class_<pcl::IterativeClosestPoint<PointT, PointT>, base<pcl::Registration<PointT, PointT>>>("IterativeClosestPoint" #PointT)   \
      .constructor<>()                                                                                                           \
      .function("setInputSource", &pcl::IterativeClosestPoint<PointT, PointT>::setInputSource)                                   \
      .function("setInputTarget", &pcl::IterativeClosestPoint<PointT, PointT>::setInputTarget)                                   \
      .function("setUseReciprocalCorrespondences", &pcl::IterativeClosestPoint<PointT, PointT>::setUseReciprocalCorrespondences) \
      .function("getUseReciprocalCorrespondences", &pcl::IterativeClosestPoint<PointT, PointT>::getUseReciprocalCorrespondences);

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr align(pcl::Registration<PointT, PointT> &registration, typename pcl::PointCloud<PointT>::Ptr &output)
{
  if (output == nullptr)
  {
    pcl::PointCloud<PointT> cloud;
    registration.align(cloud);
    return cloud.makeShared();
  }
  else
  {
    registration.align(*output);
    return output;
  }
}

template <typename PointT>
double getFitnessScore(pcl::Registration<PointT, PointT> &registration)
{
  return registration.getFitnessScore();
}

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(registration)
{
  // Bind Registration
  BIND_REGISTRATION(PointXYZ);
  BIND_REGISTRATION(PointXYZI);

  // Bind IterativeClosestPoint
  BIND_ICP(PointXYZ);
  BIND_ICP(PointXYZI);
}