#include <pcl/kdtree/kdtree_flann.h>

#include "embind.hpp"

using namespace emscripten;
using namespace pcl;

#define BIND_KdTree(r, data, PointT)                               \
  class_<pcl::KdTree<PointT>>("KdTree" BOOST_PP_STRINGIZE(PointT)) \
                                  .function("getInputCloud", &pcl::KdTree<PointT>::getInputCloud);

#define BIND_KdTreeFLANN(r, data, PointT)                                            \
  class_<pcl::KdTreeFLANN<PointT>, base<pcl::KdTree<PointT>>>(                       \
      "KdTreeFLANN" BOOST_PP_STRINGIZE(PointT))                                      \
          .constructor<bool>()                                                       \
          .function("setInputCloud", &setInputCloud<PointT>)                         \
          .function("setSortedResults", &pcl::KdTreeFLANN<PointT>::setSortedResults) \
          .function("setEpsilon", &pcl::KdTreeFLANN<PointT>::setEpsilon)             \
          .function("nearestKSearch", &nearestKSearch<PointT>)                       \
          .function("radiusSearch", &radiusSearch<PointT>);

template <typename PointT>
int nearestKSearch(pcl::KdTreeFLANN<PointT> &kdtree, const PointT &point, unsigned int k,
                   pcl::Indices &k_indices, std::vector<float> &k_distances) {
  return kdtree.nearestKSearch(point, k, k_indices, k_distances);
}

template <typename PointT>
int radiusSearch(pcl::KdTreeFLANN<PointT> &kdtree, const PointT &point, double radius,
                 pcl::Indices &k_indices, std::vector<float> &k_sqr_dists, unsigned int max_nn) {
  return kdtree.radiusSearch(point, radius, k_indices, k_sqr_dists, max_nn);
}

template <typename PointT>
void setInputCloud(pcl::KdTreeFLANN<PointT> &kdtree,
                   const typename pcl::KdTreeFLANN<PointT>::PointCloudConstPtr &cloud) {
  return kdtree.setInputCloud(cloud);
}

EMSCRIPTEN_BINDINGS(kdtree) {
  BOOST_PP_SEQ_FOR_EACH(BIND_KdTree, , XYZ_POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_KdTreeFLANN, , XYZ_POINT_TYPES);
}