#include <iostream>
#include <pcl/point_types.h>
#include <pcl/pcl_config.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <emscripten/bind.h>

// define PCLBase
#define BIND_PCL_BASE(PointT) \
  class_<pcl::PCLBase<PointT>>("PCLBase" #PointT);

// define mincut-maxflow-segementation
#define BIND_MCMF(PointT) \
  class_<pcl::MinCutSegmentation<PointT>, base<pcl::PCLBase<PointT>>>("MinCutSegmentation" #PointT) \
      .constructor<>() \
      .function("setInputCloud", &pcl::MinCutSegmentation<PointT>::setInputCloud) \
      .function("setSigma", &pcl::MinCutSegmentation<PointT>::setSigma) \
      .function("getSigma", &pcl::MinCutSegmentation<PointT>::getSigma) \
      .function("setRadius", &pcl::MinCutSegmentation<PointT>::setRadius) \
      .function("getRadius", &pcl::MinCutSegmentation<PointT>::getRadius) \
      .function("setSourceWeight", &pcl::MinCutSegmentation<PointT>::setSourceWeight) \
      .function("getSourceWeight", &pcl::MinCutSegmentation<PointT>::getSourceWeight) \
      .function("setSearchMethod", &pcl::MinCutSegmentation<PointT>::setSearchMethod) \
      .function("getSearchMethod", &pcl::MinCutSegmentation<PointT>::getSearchMethod) \
      .function("setNumberOfNeighbours", &pcl::MinCutSegmentation<PointT>::setNumberOfNeighbours) \
      .function("getNumberOfNeighbours", &pcl::MinCutSegmentation<PointT>::getNumberOfNeighbours) \
      .function("setForegroundPoints", &pcl::MinCutSegmentation<PointT>::setForegroundPoints) \
      .function("getForegroundPoints", &pcl::MinCutSegmentation<PointT>::getForegroundPoints) \
      .function("setBackgroundPoints", &pcl::MinCutSegmentation<PointT>::setBackgroundPoints) \
      .function("getBackgroundPoints", &pcl::MinCutSegmentation<PointT>::getBackgroundPoints) \
      .function("extract", &pcl::MinCutSegmentation<PointT>::extract) \
      .function("getMaxFlow", &pcl::MinCutSegmentation<PointT>::getMaxFlow) \
      .function("getGraph", &pcl::MinCutSegmentation<PointT>::getGraph) \
      .function("getColoredCloud", &pcl::MinCutSegmentation<PointT>::getColoredCloud);

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(segmentation)
{
  BIND_PCL_BASE(PointXYZ);
  BIND_PCL_BASE(PointXYZI);
  BIND_PCL_BASE(PointXYZRGB);
  BIND_PCL_BASE(PointXYZRGBA);
  BIND_PCL_BASE(PointNormal);

  BIND_MCMF(PointXYZ);
  BIND_MCMF(PointXYZI);
  BIND_MCMF(PointXYZRGB);
  BIND_MCMF(PointXYZRGBA);
  BIND_MCMF(PointNormal);
}