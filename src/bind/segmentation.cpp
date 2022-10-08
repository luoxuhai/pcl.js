#include <pcl/pcl_config.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "embind.hpp"

using namespace pcl;
using namespace emscripten;

#define BIND_MinCutSegmentation(r, data, PointT)                                                  \
  class_<pcl::MinCutSegmentation<PointT>, base<pcl::PCLBase<PointT>>>(                            \
      "MinCutSegmentation" BOOST_PP_STRINGIZE(PointT))                                            \
          .constructor<>()                                                                        \
          .function("setInputCloud", &pcl::MinCutSegmentation<PointT>::setInputCloud)             \
          .function("setSigma", &pcl::MinCutSegmentation<PointT>::setSigma)                       \
          .function("getSigma", &pcl::MinCutSegmentation<PointT>::getSigma)                       \
          .function("setRadius", &pcl::MinCutSegmentation<PointT>::setRadius)                     \
          .function("getRadius", &pcl::MinCutSegmentation<PointT>::getRadius)                     \
          .function("setSourceWeight", &pcl::MinCutSegmentation<PointT>::setSourceWeight)         \
          .function("getSourceWeight", &pcl::MinCutSegmentation<PointT>::getSourceWeight)         \
          .function("setSearchMethod", &pcl::MinCutSegmentation<PointT>::setSearchMethod)         \
          .function("getSearchMethod", &pcl::MinCutSegmentation<PointT>::getSearchMethod)         \
          .function("setNumberOfNeighbours",                                                      \
                    &pcl::MinCutSegmentation<PointT>::setNumberOfNeighbours)                      \
          .function("getNumberOfNeighbours",                                                      \
                    &pcl::MinCutSegmentation<PointT>::getNumberOfNeighbours)                      \
          .function("setForegroundPoints", &pcl::MinCutSegmentation<PointT>::setForegroundPoints) \
          .function("getForegroundPoints", &pcl::MinCutSegmentation<PointT>::getForegroundPoints) \
          .function("setBackgroundPoints", &pcl::MinCutSegmentation<PointT>::setBackgroundPoints) \
          .function("getBackgroundPoints", &pcl::MinCutSegmentation<PointT>::getBackgroundPoints) \
          .function("extract", &pcl::MinCutSegmentation<PointT>::extract, allow_raw_pointers())   \
          .function("getMaxFlow", &pcl::MinCutSegmentation<PointT>::getMaxFlow)                   \
          .function("getColoredCloud", &pcl::MinCutSegmentation<PointT>::getColoredCloud);

EMSCRIPTEN_BINDINGS(segmentation) {
  BOOST_PP_SEQ_FOR_EACH(BIND_MinCutSegmentation, , XYZ_POINT_TYPES);

  class_<pcl::SACSegmentation<pcl::PointXYZ>, base<pcl::PCLBase<pcl::PointXYZ>>>("SACSegmentation")
      .constructor<bool>()
      .function("setModelType", &pcl::SACSegmentation<pcl::PointXYZ>::setModelType)
      .function("setMethodType", &pcl::SACSegmentation<pcl::PointXYZ>::setMethodType)
      .function("setDistanceThreshold", &pcl::SACSegmentation<pcl::PointXYZ>::setDistanceThreshold)
      .function("setMaxIterations", &pcl::SACSegmentation<pcl::PointXYZ>::setMaxIterations)
      .function("setProbability", &pcl::SACSegmentation<pcl::PointXYZ>::setProbability)
      .function("setRadiusLimits", &pcl::SACSegmentation<pcl::PointXYZ>::setRadiusLimits)
      .function("setSamplesMaxDist", &pcl::SACSegmentation<pcl::PointXYZ>::setSamplesMaxDist)
      .function("setEpsAngle", &pcl::SACSegmentation<pcl::PointXYZ>::setEpsAngle)
      .function("setOptimizeCoefficients",
                &pcl::SACSegmentation<pcl::PointXYZ>::setOptimizeCoefficients)
      .function("segment", &pcl::SACSegmentation<pcl::PointXYZ>::segment);

  register_vector<PointIndices>("VectorPointIndices");
}