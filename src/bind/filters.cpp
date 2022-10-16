#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>

#include "embind.hpp"

#define BIND_PassThrough(r, data, PointT)                                                \
  class_<pcl::PassThrough<PointT>, base<pcl::FilterIndices<PointT>>>(                    \
      "PassThrough" BOOST_PP_STRINGIZE(PointT))                                          \
          .constructor<bool>()                                                           \
          .function("setFilterFieldName", &pcl::PassThrough<PointT>::setFilterFieldName) \
          .function("getFilterFieldName", &pcl::PassThrough<PointT>::getFilterFieldName) \
          .function("setFilterLimits", &pcl::PassThrough<PointT>::setFilterLimits)       \
          .function("getFilterLimits", &getFilterLimits<PointT>);

#define BIND_FilterIndices(r, data, PointT)                                            \
  class_<pcl::FilterIndices<PointT>, base<pcl::Filter<PointT>>>(                       \
      "FilterIndices" BOOST_PP_STRINGIZE(PointT))                                      \
          .function("setNegative", &pcl::FilterIndices<PointT>::setNegative)           \
          .function("getNegative", &pcl::FilterIndices<PointT>::getNegative)           \
          .function("setKeepOrganized", &pcl::FilterIndices<PointT>::setKeepOrganized) \
          .function("getKeepOrganized", &pcl::FilterIndices<PointT>::getKeepOrganized) \
          .function("setUserFilterValue", &pcl::FilterIndices<PointT>::setUserFilterValue);

#define BIND_Filter(r, data, PointT)                                 \
  class_<pcl::Filter<PointT>, base<pcl::PCLBase<PointT>>>(           \
      "Filter" BOOST_PP_STRINGIZE(PointT))                           \
          .function("getRemovedIndices", &getRemovedIndices<PointT>) \
          .function("filter", &pcl::Filter<PointT>::filter);

#define BIND_VoxelGrid(r, data, PointT)                                                    \
  class_<pcl::VoxelGrid<PointT>, base<pcl::Filter<PointT>>>(                               \
      "VoxelGrid" BOOST_PP_STRINGIZE(PointT))                                              \
          .constructor()                                                                   \
          .function("setLeafSize", select_overload<void(float, float, float)>(             \
                                       &pcl::VoxelGrid<PointT>::setLeafSize))              \
          .function("setDownsampleAllData", &pcl::VoxelGrid<PointT>::setDownsampleAllData) \
          .function("getDownsampleAllData", &pcl::VoxelGrid<PointT>::getDownsampleAllData) \
          .function("setMinimumPointsNumberPerVoxel",                                      \
                    &pcl::VoxelGrid<PointT>::setMinimumPointsNumberPerVoxel)               \
          .function("getMinimumPointsNumberPerVoxel",                                      \
                    &pcl::VoxelGrid<PointT>::getMinimumPointsNumberPerVoxel);

#define BIND_StatisticalOutlierRemoval(r, data, PointT)                             \
  class_<pcl::StatisticalOutlierRemoval<PointT>, base<pcl::FilterIndices<PointT>>>( \
      "StatisticalOutlierRemoval" BOOST_PP_STRINGIZE(PointT))                       \
          .constructor<bool>()                                                      \
          .function("setMeanK", &pcl::StatisticalOutlierRemoval<PointT>::setMeanK)  \
          .function("getMeanK", &pcl::StatisticalOutlierRemoval<PointT>::getMeanK)  \
          .function("setStddevMulThresh",                                           \
                    &pcl::StatisticalOutlierRemoval<PointT>::setStddevMulThresh)    \
          .function("getStddevMulThresh",                                           \
                    &pcl::StatisticalOutlierRemoval<PointT>::getStddevMulThresh);

#define BIND_RadiusOutlierRemoval(r, data, PointT)                                          \
  class_<pcl::RadiusOutlierRemoval<PointT>, base<pcl::FilterIndices<PointT>>>(              \
      "RadiusOutlierRemoval" BOOST_PP_STRINGIZE(PointT))                                    \
          .constructor<bool>()                                                              \
          .function("setRadiusSearch", &pcl::RadiusOutlierRemoval<PointT>::setRadiusSearch) \
          .function("getRadiusSearch", &pcl::RadiusOutlierRemoval<PointT>::getRadiusSearch) \
          .function("setMinNeighborsInRadius",                                              \
                    &pcl::RadiusOutlierRemoval<PointT>::setMinNeighborsInRadius)            \
          .function("getMinNeighborsInRadius",                                              \
                    &pcl::RadiusOutlierRemoval<PointT>::getMinNeighborsInRadius);

#define BIND_UniformSampling(r, data, PointT)                      \
  class_<pcl::UniformSampling<PointT>, base<pcl::Filter<PointT>>>( \
      "UniformSampling" BOOST_PP_STRINGIZE(PointT))                \
          .constructor<bool>()                                     \
          .function("setRadiusSearch", &pcl::UniformSampling<PointT>::setRadiusSearch);

#define BIND_RandomSample(r, data, PointT)                              \
  class_<pcl::RandomSample<PointT>, base<pcl::FilterIndices<PointT>>>(  \
      "RandomSample" BOOST_PP_STRINGIZE(PointT))                        \
          .constructor<bool>()                                          \
          .function("setSample", &pcl::RandomSample<PointT>::setSample) \
          .function("getSample", &pcl::RandomSample<PointT>::getSample) \
          .function("setSeed", &pcl::RandomSample<PointT>::setSeed)     \
          .function("getSeed", &pcl::RandomSample<PointT>::getSeed);

#define BIND_GridMinimum(r, data, PointT)                                      \
  class_<pcl::GridMinimum<PointT>, base<pcl::FilterIndices<PointT>>>(          \
      "GridMinimum" BOOST_PP_STRINGIZE(PointT))                                \
          .constructor<float>()                                                \
          .function("setResolution", &pcl::GridMinimum<PointT>::setResolution) \
          .function("getResolution", &pcl::GridMinimum<PointT>::getResolution);

#define BIND_LocalMaximum(r, data, PointT)                              \
  class_<pcl::LocalMaximum<PointT>, base<pcl::FilterIndices<PointT>>>(  \
      "LocalMaximum" BOOST_PP_STRINGIZE(PointT))                        \
          .constructor<bool>()                                          \
          .function("setRadius", &pcl::LocalMaximum<PointT>::setRadius) \
          .function("getRadius", &pcl::LocalMaximum<PointT>::getRadius);

#define BIND_ApproximateVoxelGrid(r, data, PointT)                                       \
  class_<pcl::ApproximateVoxelGrid<PointT>, base<pcl::Filter<PointT>>>(                  \
      "ApproximateVoxelGrid" BOOST_PP_STRINGIZE(PointT))                                 \
          .constructor()                                                                 \
          .function("setLeafSize", select_overload<void(float, float, float)>(           \
                                       &pcl::ApproximateVoxelGrid<PointT>::setLeafSize)) \
          .function("setDownsampleAllData",                                              \
                    &pcl::ApproximateVoxelGrid<PointT>::setDownsampleAllData)            \
          .function("getDownsampleAllData",                                              \
                    &pcl::ApproximateVoxelGrid<PointT>::getDownsampleAllData);

#define BIND_removeNaNFromPointCloud(r, data, PointT)                                           \
  function("removeNaNFromPointCloud" BOOST_PP_STRINGIZE(PointT), select_overload<void(const pcl::PointCloud<PointT> &, pcl::PointCloud<PointT> &, \
                                    pcl::Indices &)>(&pcl::removeNaNFromPointCloud<PointT>));

#define BIND_removeNaNNormalsFromPointCloud(r, data, PointT) \
  function("removeNaNNormalsFromPointCloud" BOOST_PP_STRINGIZE(PointT), &pcl::removeNaNNormalsFromPointCloud<PointT>);

struct FilterLimits {
  float min;
  float max;
};

template <typename PointT>
FilterLimits getFilterLimits(pcl::PassThrough<PointT> &passThrough) {
  float limit_min;
  float limit_max;
  passThrough.getFilterLimits(limit_min, limit_max);

  FilterLimits limits = {.min = limit_min, .max = limit_max};

  return limits;
}

template <typename PointT>
pcl::Indices getRemovedIndices(pcl::Filter<PointT> &filter) {
  pcl::PointIndices pointIndices;
  filter.getRemovedIndices(pointIndices);
  return pointIndices.indices;
}

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(filters) {
  // Bind PassThrough
  BOOST_PP_SEQ_FOR_EACH(BIND_PassThrough, , (PointXYZ));

  // Bind FilterIndices
  BOOST_PP_SEQ_FOR_EACH(BIND_FilterIndices, , POINT_TYPES);

  // Bind Filter
  BOOST_PP_SEQ_FOR_EACH(BIND_Filter, , POINT_TYPES);

  // Bind VoxelGrid
  BOOST_PP_SEQ_FOR_EACH(BIND_VoxelGrid, , (PointXYZ));

  // Bind StatisticalOutlierRemoval
  BOOST_PP_SEQ_FOR_EACH(BIND_StatisticalOutlierRemoval, , (PointXYZ));

  // Bind RadiusOutlierRemoval
  BOOST_PP_SEQ_FOR_EACH(BIND_RadiusOutlierRemoval, , (PointXYZ));

  // Bind UniformSampling
  BOOST_PP_SEQ_FOR_EACH(BIND_UniformSampling, , (PointXYZ));

  // Bind RandomSample
  BOOST_PP_SEQ_FOR_EACH(BIND_RandomSample, , POINT_TYPES);

  // Bind GridMinimum
  BOOST_PP_SEQ_FOR_EACH(BIND_GridMinimum, , (PointXYZ));

  // Bind LocalMaximum
  BOOST_PP_SEQ_FOR_EACH(BIND_LocalMaximum, , (PointXYZ));

  // Bind ApproximateVoxelGrid
  BOOST_PP_SEQ_FOR_EACH(BIND_ApproximateVoxelGrid, , (PointXYZ));

  value_array<FilterLimits>("FilterLimits").element(&FilterLimits::min).element(&FilterLimits::max);

  // Bind removeNaNFromPointCloud
  BOOST_PP_SEQ_FOR_EACH(BIND_removeNaNFromPointCloud, , (PointXYZ));

  // Bind removeNaNNormalsFromPointCloud
  BOOST_PP_SEQ_FOR_EACH(BIND_removeNaNNormalsFromPointCloud, , NORMAL_POINT_TYPES);
}
