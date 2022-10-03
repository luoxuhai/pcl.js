#include <pcl/search/flann_search.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include "embind.hpp"

using namespace pcl;
using namespace emscripten;

#define BIND_SEARCH_Search(r, data, PointT)                                             \
  class_<pcl::search::Search<PointT>>(                                                  \
      "Search" BOOST_PP_STRINGIZE(PointT))                                              \
          .function("getName", &pcl::search::Search<PointT>::getName)                   \
          .function("getSortedResults", &pcl::search::Search<PointT>::getSortedResults) \
          .function("getInputCloud", &pcl::search::Search<PointT>::getInputCloud)       \
          .smart_ptr<pcl::search::Search<PointT>::ConstPtr>(                            \
              "SearchConstPtr" BOOST_PP_STRINGIZE(PointT))                              \
                  .smart_ptr<pcl::search::Search<PointT>::Ptr>(                         \
                      "SearchPtr" BOOST_PP_STRINGIZE(PointT));

#define BIND_SEARCH_KdTree(r, data, PointT)                                                     \
  class_<pcl::search::KdTree<PointT>, base<pcl::search::Search<PointT>>>(                       \
      "SearchKdTree" BOOST_PP_STRINGIZE(PointT))                                                \
          .smart_ptr_constructor<pcl::search::KdTree<PointT>::Ptr, bool &&>(                    \
              "SearchKdTree" BOOST_PP_STRINGIZE(PointT),                                        \
                                                &std::make_shared<pcl::search::KdTree<PointT>>) \
                  .function("setInputCloud", &setInputCloud<PointT>)                            \
                  .function("setSortedResults", &pcl::search::KdTree<PointT>::setSortedResults) \
                  .function("setEpsilon", &pcl::search::KdTree<PointT>::setEpsilon)             \
                  .function("nearestKSearch", &nearestKSearchKdTree<PointT>)                    \
                  .function("radiusSearch", &radiusSearchKdTree<PointT>)                        \
                  .smart_ptr<pcl::search::KdTree<PointT>::ConstPtr>(                            \
                      "SearchKdTreeConstPtr" BOOST_PP_STRINGIZE(PointT));

#define BIND_SEARCH_FlannSearch(r, data, PointT)                                         \
  class_<pcl::search::FlannSearch<PointT>, base<pcl::search::Search<PointT>>>(           \
      "FlannSearch" BOOST_PP_STRINGIZE(PointT))                                          \
          .smart_ptr_constructor<pcl::search::FlannSearch<PointT>::Ptr, bool &&>(        \
              "FlannSearch" BOOST_PP_STRINGIZE(PointT), &std::make_shared<pcl::search::FlannSearch<PointT>>)          \
                  .function("setInputCloud", &setInputCloudKdTreeFLANN<PointT>)          \
                  .function("setEpsilon", &pcl::search::FlannSearch<PointT>::setEpsilon) \
                  .function("getEpsilon", &pcl::search::FlannSearch<PointT>::getEpsilon) \
                  .function("setChecks", &pcl::search::FlannSearch<PointT>::setChecks)   \
                  .function("getChecks", &pcl::search::FlannSearch<PointT>::getChecks)   \
                  .function("nearestKSearch", &nearestKSearchFlannSearch<PointT>)        \
                  .function("radiusSearch", &radiusSearchFlannSearch<PointT>)            \
                  .smart_ptr<pcl::search::FlannSearch<PointT>::ConstPtr>(                \
                      "FlannSearchConstPtr" BOOST_PP_STRINGIZE(PointT));

template <typename PointT>
int nearestKSearchKdTree(pcl::search::KdTree<PointT> &kdtree, const PointT &point, unsigned int k,
                         pcl::Indices &k_indices, std::vector<float> &k_distances) {
  return kdtree.nearestKSearch(point, k, k_indices, k_distances);
}

template <typename PointT>
int radiusSearchKdTree(pcl::search::KdTree<PointT> &kdtree, const PointT &point, double radius,
                       pcl::Indices &k_indices, std::vector<float> &k_sqr_dists,
                       unsigned int max_nn) {
  return kdtree.radiusSearch(point, radius, k_indices, k_sqr_dists, max_nn);
}

template <typename PointT>
int nearestKSearchFlannSearch(pcl::search::FlannSearch<PointT> &kdtree, const PointT &point,
                              unsigned int k, pcl::Indices &k_indices,
                              std::vector<float> &k_distances) {
  return kdtree.nearestKSearch(point, k, k_indices, k_distances);
}

template <typename PointT>
int radiusSearchFlannSearch(pcl::search::FlannSearch<PointT> &kdtree, const PointT &point,
                            double radius, pcl::Indices &k_indices, std::vector<float> &k_sqr_dists,
                            unsigned int max_nn) {
  return kdtree.radiusSearch(point, radius, k_indices, k_sqr_dists, max_nn);
}

template <typename PointT>
void setInputCloud(pcl::search::KdTree<PointT> &kdtree,
                   const typename pcl::search::KdTree<PointT>::PointCloudConstPtr &cloud) {
  return kdtree.setInputCloud(cloud);
}

template <typename PointT>
void setInputCloudKdTreeFLANN(
    pcl::search::FlannSearch<PointT> &kdtree,
    const typename pcl::search::FlannSearch<PointT>::PointCloudConstPtr &cloud) {
  return kdtree.setInputCloud(cloud);
}

EMSCRIPTEN_BINDINGS(search) {
  BOOST_PP_SEQ_FOR_EACH(BIND_SEARCH_Search, , XYZ_POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_SEARCH_KdTree, , XYZ_POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_SEARCH_FlannSearch, , XYZ_POINT_TYPES);
}