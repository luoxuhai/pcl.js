#include <pcl/point_types.h>
#include <pcl/pcl_config.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <emscripten/bind.h>

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

// template <typename PointT>
// void setForegroundPoints(pcl::MinCutSegmentation<PointT> &segmentation, typename pcl::PointCloud<PointT>::Ptr &foreground_points) {
//   segmentation.setForegroundPoints(foreground_points);
// }

// template <typename PointT>
// void setBackgroundPoints(pcl::MinCutSegmentation<PointT> &segmentation, typename pcl::PointCloud<PointT>::Ptr &background_points) {
//   segmentation.setBackgroundPoints(background_points);
// }

template <typename PointT>
typename std::vector<pcl::PointIndices>
extract(pcl::MinCutSegmentation<PointT> &segmentation, std::vector<pcl::PointIndices>& clusters)
{
  segmentation.extract(clusters);
  return clusters;
}

using namespace pcl;
using namespace emscripten;

std::vector<PointIndices> returnVectorPointIndices () {
  std::vector<PointIndices> v(2);
  return v;
}

EMSCRIPTEN_BINDINGS(segmentation)
{
  BIND_MCMF(PointXYZ);
  BIND_MCMF(PointXYZI);
  BIND_MCMF(PointXYZRGB);
  BIND_MCMF(PointXYZRGBA);
  BIND_MCMF(PointNormal);
  function("returnVectorPointIndices", &returnVectorPointIndices);
  register_vector<PointIndices>("vector<PointIndices>");
}