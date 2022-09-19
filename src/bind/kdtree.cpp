#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <emscripten/bind.h>

#define BIND_KDTREE(PointT)                     \
  class_<pcl::KdTree<PointT>>("KdTree" #PointT) \
      .function("getInputCloud", &pcl::KdTree<PointT>::getInputCloud);

#define BIND_KDTREE_FLANN(PointT)                                                    \
  class_<pcl::KdTreeFLANN<PointT>, base<pcl::KdTree<PointT>>>("KdTreeFLANN" #PointT) \
      .constructor<bool>()                                                           \
      .function("setInputCloud", &setInputCloud<PointT>)                             \
      .function("setSortedResults", &pcl::KdTreeFLANN<PointT>::setSortedResults)     \
      .function("setEpsilon", &pcl::KdTreeFLANN<PointT>::setEpsilon)                 \
      .function("nearestKSearch", &nearestKSearch<PointT>)                           \
      .function("radiusSearch", &radiusSearch<PointT>);

template <typename PointT>
int nearestKSearch(pcl::KdTreeFLANN<PointT> &kdtree, const PointT &point, unsigned int k,
                   pcl::Indices &k_indices,
                   std::vector<float> &k_distances)
{
  return kdtree.nearestKSearch(point, k, k_indices, k_distances);
}

template <typename PointT>
int radiusSearch(pcl::KdTreeFLANN<PointT> &kdtree, const PointT &point, double radius, pcl::Indices &k_indices,
                 std::vector<float> &k_sqr_dists, unsigned int max_nn)
{
  return kdtree.radiusSearch(point, radius, k_indices, k_sqr_dists, max_nn);
}

template <typename PointT>
void setInputCloud(pcl::KdTreeFLANN<PointT> &kdtree, const typename pcl::KdTreeFLANN<PointT>::PointCloudConstPtr &cloud)
{
  return kdtree.setInputCloud(cloud);
}

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(kdtree)
{
  BIND_KDTREE(PointXYZ);
  BIND_KDTREE(PointXYZI);
  BIND_KDTREE(PointXYZRGB);
  BIND_KDTREE(PointXYZRGBA);
  BIND_KDTREE(PointNormal);

  BIND_KDTREE_FLANN(PointXYZ);
  BIND_KDTREE_FLANN(PointXYZI);
  BIND_KDTREE_FLANN(PointXYZRGB);
  BIND_KDTREE_FLANN(PointXYZRGBA);
  BIND_KDTREE_FLANN(PointNormal);
}