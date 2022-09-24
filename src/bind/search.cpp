#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <emscripten/bind.h>

#define BIND_SEARCH(PointT)                                                         \
  class_<pcl::search::Search<PointT>>("Search" #PointT)                             \
      .function("getName", &pcl::search::Search<PointT>::getName)                   \
      .function("getSortedResults", &pcl::search::Search<PointT>::getSortedResults) \
      .function("getInputCloud", &pcl::search::Search<PointT>::getInputCloud)       \
      .smart_ptr<pcl::search::Search<PointT>::ConstPtr>("SearchConstPtr" #PointT)   \
      .smart_ptr<pcl::search::Search<PointT>::Ptr>("SearchPtr" #PointT);

#define BIND_SEARCH_KDTREE(PointT)                                                               \
  class_<pcl::search::KdTree<PointT>, base<pcl::search::Search<PointT>>>("SearchKdTree" #PointT) \
      .constructor<bool>()                                                                       \
      .function("setInputCloud", &setInputCloud<PointT>)                                         \
      .function("setSortedResults", &pcl::search::KdTree<PointT>::setSortedResults)              \
      .function("setEpsilon", &pcl::search::KdTree<PointT>::setEpsilon)                          \
      .function("nearestKSearch", &nearestKSearch<PointT>)                                       \
      .function("radiusSearch", &radiusSearch<PointT>)                                           \
      .smart_ptr<pcl::search::KdTree<PointT>::ConstPtr>("SearchKdTreeConstPtr" #PointT)          \
      .smart_ptr<pcl::search::KdTree<PointT>::Ptr>("SearchKdTreePtr" #PointT);

template <typename PointT>
int nearestKSearch(pcl::search::KdTree<PointT> &kdtree, const PointT &point, unsigned int k,
                   pcl::Indices &k_indices,
                   std::vector<float> &k_distances)
{
  return kdtree.nearestKSearch(point, k, k_indices, k_distances);
}

template <typename PointT>
int radiusSearch(pcl::search::KdTree<PointT> &kdtree, const PointT &point, double radius, pcl::Indices &k_indices,
                 std::vector<float> &k_sqr_dists, unsigned int max_nn)
{
  return kdtree.radiusSearch(point, radius, k_indices, k_sqr_dists, max_nn);
}

template <typename PointT>
void setInputCloud(pcl::search::KdTree<PointT> &kdtree, const typename pcl::search::KdTree<PointT>::PointCloudConstPtr &cloud)
{
  return kdtree.setInputCloud(cloud);
}

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(search)
{
  BIND_SEARCH(PointXYZ);
  BIND_SEARCH(PointXYZI);
  BIND_SEARCH(PointXYZRGB);
  BIND_SEARCH(PointXYZRGBA);
  BIND_SEARCH(PointNormal);

  BIND_SEARCH_KDTREE(PointXYZ);
  BIND_SEARCH_KDTREE(PointXYZI);
  BIND_SEARCH_KDTREE(PointXYZRGB);
  BIND_SEARCH_KDTREE(PointXYZRGBA);
  BIND_SEARCH_KDTREE(PointNormal);
}