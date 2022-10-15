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

#define BIND_SACSegmentation(r, data, PointT)                                                 \
  class_<pcl::SACSegmentation<PointT>, base<pcl::PCLBase<PointT>>>(                              \
      "SACSegmentation" BOOST_PP_STRINGIZE(PointT))                                              \
          .constructor<bool>()                                                                   \
          .function("setModelType", &pcl::SACSegmentation<PointT>::setModelType)                 \
          .function("setMethodType", &pcl::SACSegmentation<PointT>::setMethodType)               \
          .function("setDistanceThreshold", &pcl::SACSegmentation<PointT>::setDistanceThreshold) \
          .function("setMaxIterations", &pcl::SACSegmentation<PointT>::setMaxIterations)         \
          .function("setProbability", &pcl::SACSegmentation<PointT>::setProbability)             \
          .function("setRadiusLimits", &pcl::SACSegmentation<PointT>::setRadiusLimits)           \
          .function("setSamplesMaxDist", &pcl::SACSegmentation<PointT>::setSamplesMaxDist)       \
          .function("setEpsAngle", &pcl::SACSegmentation<PointT>::setEpsAngle)                   \
          .function("setOptimizeCoefficients",                                                   \
                    &pcl::SACSegmentation<PointT>::setOptimizeCoefficients)                      \
          .function("segment", &pcl::SACSegmentation<PointT>::segment);

EMSCRIPTEN_BINDINGS(segmentation) {
  BOOST_PP_SEQ_FOR_EACH(BIND_MinCutSegmentation, , XYZ_POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_SACSegmentation, , (PointXYZ));

  register_vector<PointIndices>("VectorPointIndices");
}