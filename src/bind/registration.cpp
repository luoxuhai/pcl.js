#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <emscripten/bind.h>

using namespace emscripten;

typedef pcl::PointXYZ PointCloudXYZ;

double getFitnessScore(pcl::Registration<PointCloudXYZ, PointCloudXYZ> &registration)
{
  return registration.getFitnessScore();
}

pcl::PointCloud<PointCloudXYZ> align(pcl::Registration<PointCloudXYZ, PointCloudXYZ> &registration)
{
  pcl::PointCloud<PointCloudXYZ> cloud;

  registration.align(cloud);

  return cloud;
}

EMSCRIPTEN_BINDINGS(registration)
{
  class_<pcl::Registration<PointCloudXYZ, PointCloudXYZ>>("Registration")
      .function("hasConverged", &pcl::Registration<PointCloudXYZ, PointCloudXYZ>::hasConverged)
      .function("getFinalTransformation", &pcl::Registration<PointCloudXYZ, PointCloudXYZ>::getFinalTransformation)
      .function("getFitnessScore", &getFitnessScore)
      .function("align", &align);

  // IterativeClosestPoint

  class_<pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>, base<pcl::Registration<PointCloudXYZ, PointCloudXYZ>>>("IterativeClosestPoint")
      .constructor<>()
      .function("setInputSource", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::setInputSource)
      .function("setInputTarget", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::setInputTarget)
      .function("setUseReciprocalCorrespondences", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::setUseReciprocalCorrespondences)
      .function("getUseReciprocalCorrespondences", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::getUseReciprocalCorrespondences);
}