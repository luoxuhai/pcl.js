#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/normal_3d.h>

#include "embind.hpp"

#define BIND_Feature(PointInT, PointOutT)                                                     \
  class_<pcl::Feature<PointInT, PointOutT>, base<pcl::PCLBase<PointInT>>>(                    \
      "Feature" #PointInT #PointOutT)                                                         \
      .function("setSearchSurface", &pcl::Feature<PointInT, PointOutT>::setSearchSurface)     \
      .function("setSearchMethod", &pcl::Feature<PointInT, PointOutT>::setSearchMethod)       \
      .function("getSearchParameter", &pcl::Feature<PointInT, PointOutT>::getSearchParameter) \
      .function("setKSearch", &pcl::Feature<PointInT, PointOutT>::setKSearch)                 \
      .function("getKSearch", &pcl::Feature<PointInT, PointOutT>::getKSearch)                 \
      .function("setRadiusSearch", &pcl::Feature<PointInT, PointOutT>::setRadiusSearch)       \
      .function("getRadiusSearch", &pcl::Feature<PointInT, PointOutT>::getRadiusSearch)       \
      .function("compute", &pcl::Feature<PointInT, PointOutT>::compute);

#define BIND_FeatureFromNormals(PointInT, PointNT, PointOutT)   \
  class_<pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>, \
         base<pcl::Feature<PointInT, PointOutT>>>(              \
      "FeatureFromNormals" #PointInT #PointNT #PointOutT)       \
      .function("setInputNormals",                              \
                &pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>::setInputNormals);

#define BIND_FPFHEstimation(PointInT, PointNT, PointOutT)              \
  class_<pcl::FPFHEstimation<PointInT, PointNT, PointOutT>,            \
         base<pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>>>( \
      "FPFHEstimation" #PointInT #PointNT #PointOutT)                  \
      .constructor()                                                   \
      .function("setNrSubdivisions",                                   \
                &pcl::FPFHEstimation<PointInT, PointNT, PointOutT>::setNrSubdivisions);

#define BIND_NormalEstimation(PointInT, PointOutT)                                             \
  class_<pcl::NormalEstimation<PointInT, PointOutT>, base<pcl::Feature<PointInT, PointOutT>>>( \
      "NormalEstimation" #PointInT #PointOutT)                                                 \
      .constructor()                                                                           \
      .function("setInputCloud", &pcl::NormalEstimation<PointInT, PointOutT>::setInputCloud)   \
      .function("setViewPoint", &pcl::NormalEstimation<PointInT, PointOutT>::setViewPoint)     \
      .function("useSensorOriginAsViewPoint",                                                  \
                &pcl::NormalEstimation<PointInT, PointOutT>::useSensorOriginAsViewPoint);

EMSCRIPTEN_BINDINGS(features) {
  BIND_Feature(PointXYZ, FPFHSignature33);

  BIND_Feature(PointXYZ, Normal);

  BIND_FeatureFromNormals(PointXYZ, Normal, FPFHSignature33);

  BIND_FPFHEstimation(PointXYZ, Normal, FPFHSignature33);

  BIND_NormalEstimation(PointXYZ, Normal);
}