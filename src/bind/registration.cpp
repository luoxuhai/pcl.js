#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <emscripten/bind.h>

using namespace emscripten;

typedef pcl::PointXYZ PointCloudXYZ;

EMSCRIPTEN_BINDINGS(registration)
{
  class_<pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>>("IterativeClosestPoint")
      .constructor<>()
      .function("setInputSource", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::setInputSource)
      .function("setInputTarget", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::setInputTarget)
      .function("hasConverged", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::hasConverged)
      .function("getFinalTransformation", &pcl::IterativeClosestPoint<PointCloudXYZ, PointCloudXYZ>::getFinalTransformation);
}