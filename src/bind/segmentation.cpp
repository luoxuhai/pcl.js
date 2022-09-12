#include <iostream>
#include <pcl/point_types.h>
<<<<<<< HEAD
#include <pcl/pcl_config.h>
=======
>>>>>>> + add binding for min cut segmentation
#include <pcl/segmentation/min_cut_segmentation.h>
#include <emscripten/bind.h>

// define PCLBase
<<<<<<< HEAD
#define BIND_PCL_BASE(PointT) \
  class_<pcl::PCLBase<PointT>>("PCLBase" #PointT);
=======
#define BIND_PCLBASE
  class_<pcl
>>>>>>> + add binding for min cut segmentation

// define mincut-maxflow-segementation
#define BIND_MCMF(PointT) \
  class_<pcl::MinCutSegmentation<PointT>, base<pcl::PCLBase<PointT>>>("MinCutSegmentation" #PointT) \
      .constructor<>() \
      .function("setInputCloud", &pcl::MinCutSegmentation<PointT>::setInputCloud) \
<<<<<<< HEAD
      .function("setSigma", &pcl::MinCutSegmentation<PointT>::setSigma) \
      .function("getSigma", &pcl::MinCutSegmentation<PointT>::getSigma) \
=======
      .function("setSigma", &pcl::MinCutSegementation<PointT>::setSigMa) \
      .function("getSigma", &pcl::MinCutSegementation<PointT>::getSigMa) \
>>>>>>> + add binding for min cut segmentation
      .function("setRadius", &pcl::MinCutSegmentation<PointT>::setRadius) \
      .function("getRadius", &pcl::MinCutSegmentation<PointT>::getRadius) \
      .function("setSourceWeight", &pcl::MinCutSegmentation<PointT>::setSourceWeight) \
      .function("getSourceWeight", &pcl::MinCutSegmentation<PointT>::getSourceWeight) \
      .function("setSearchMethod", &pcl::MinCutSegmentation<PointT>::setSearchMethod) \
      .function("getSearchMethod", &pcl::MinCutSegmentation<PointT>::getSearchMethod) \
<<<<<<< HEAD
      .function("setNumberOfNeighbours", &pcl::MinCutSegmentation<PointT>::setNumberOfNeighbours) \
      .function("getNumberOfNeighbours", &pcl::MinCutSegmentation<PointT>::getNumberOfNeighbours) \
=======
      .function("setNumberOfNeighbors", &pcl::MinCutSegmentation<PointT>::setNumberOfNeighbors) \
      .function("getNumberOfNeighbors", &pcl::MinCutSegmentation<PointT>::getNumberOfNeighbors) \
>>>>>>> + add binding for min cut segmentation
      .function("setForegroundPoints", &pcl::MinCutSegmentation<PointT>::setForegroundPoints) \
      .function("getForegroundPoints", &pcl::MinCutSegmentation<PointT>::getForegroundPoints) \
      .function("setBackgroundPoints", &pcl::MinCutSegmentation<PointT>::setBackgroundPoints) \
      .function("getBackgroundPoints", &pcl::MinCutSegmentation<PointT>::getBackgroundPoints) \
      .function("extract", &pcl::MinCutSegmentation<PointT>::extract) \
      .function("getMaxFlow", &pcl::MinCutSegmentation<PointT>::getMaxFlow) \
      .function("getGraph", &pcl::MinCutSegmentation<PointT>::getGraph) \
      .function("getColoredCloud", &pcl::MinCutSegmentation<PointT>::getColoredCloud);
<<<<<<< HEAD

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(segmentation)
{
  BIND_PCL_BASE(PointXYZ);
  BIND_PCL_BASE(PointXYZI);
  BIND_PCL_BASE(PointXYZRGB);
  BIND_PCL_BASE(PointXYZRGBA);
  BIND_PCL_BASE(Normal);
  BIND_PCL_BASE(PointNormal);

  BIND_MCMF(PointXYZ);
  BIND_MCMF(PointXYZI);
  BIND_MCMF(PointXYZRGB);
  BIND_MCMF(PointXYZRGBA);
  BIND_MCMF(Normal);
  BIND_MCMF(PointNormal);
}
=======
>>>>>>> + add binding for min cut segmentation
