#include <pcl/ModelCoefficients.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/search/kdtree.h>

#include "embind.hpp"

using namespace pcl;

const std::string VERSION = PCL_VERSION_PRETTY;

#define BIND_PointCloud(r, data, PointT)                                                      \
  class_<PointCloud<PointT>>(                                                                 \
      "PointCloud" BOOST_PP_STRINGIZE(PointT))                                                \
          .smart_ptr_constructor<PointCloud<PointT>::Ptr>(                                    \
              "PointCloud" BOOST_PP_STRINGIZE(PointT), &std::make_shared<PointCloud<PointT>>) \
                  .property("width", &PointCloud<PointT>::width)                              \
                  .property("height", &PointCloud<PointT>::height)                            \
                  .property("points", &PointCloud<PointT>::points)                            \
                  .property("is_dense", &PointCloud<PointT>::is_dense)                        \
                  .property("header", &PointCloud<PointT>::header)                            \
                  .function("isOrganized", &PointCloud<PointT>::isOrganized)                  \
                  .function("resize", select_overload<void(index_t, const PointT &)>(         \
                                          &PointCloud<PointT>::resize))                       \
                  .function("push_back", &PointCloud<PointT>::push_back)                      \
                  .function("clear", &PointCloud<PointT>::clear)                              \
                  .smart_ptr<PointCloud<PointT>::ConstPtr>(                                   \
                      "PointCloudConstPtr" BOOST_PP_STRINGIZE(PointT));

#define BIND_Points(r, data, PointT)                              \
  register_vector_plus<PointT, Eigen::aligned_allocator<PointT>>( \
      "Points" BOOST_PP_STRINGIZE(PointT));

#define BIND_PCLBase(r, data, PointT)                                                     \
  class_<PCLBase<PointT>>("PCLBase" BOOST_PP_STRINGIZE(PointT))                           \
                              .function("setInputCloud", &PCLBase<PointT>::setInputCloud) \
                              .function("getInputCloud", &PCLBase<PointT>::getInputCloud);

#define BIND_computeCloudResolution(r, data, PointT) \
  function("computeCloudResolution" BOOST_PP_STRINGIZE(PointT), &computeCloudResolution<PointT>);

template <typename PointT>
double computeCloudResolution(const typename PointCloud<PointT>::ConstPtr &cloud) {
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> squaredDistances(2);
  typename search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>());
  tree->setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i) {
    if (!std::isfinite((*cloud)[i].x)) continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree->nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2) {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0) resolution /= numberOfPoints;

  return resolution;
}

template <typename PointInT>
void toXYZPointCloud(const pcl::PointCloud<PointInT> &cloud_in,
                     pcl::PointCloud<pcl::PointXYZ> &cloud_out) {
  pcl::copyPointCloud(cloud_in, cloud_out);
}

#define BIND_toXYZPointCloud(r, data, PointT) \
  function("toXYZPointCloud" BOOST_PP_STRINGIZE(PointT), &toXYZPointCloud<PointT>);

EMSCRIPTEN_BINDINGS(common) {
  constant("PCL_VERSION", VERSION);

  BOOST_PP_SEQ_FOR_EACH(BIND_PointCloud, , POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_Points, , POINT_TYPES);

  BOOST_PP_SEQ_FOR_EACH(BIND_PCLBase, , POINT_TYPES);

  register_vector_plus<float>("VectorFloat");
  register_vector_plus<index_t>("Indices");

  BOOST_PP_SEQ_FOR_EACH(BIND_computeCloudResolution, , XYZ_POINT_TYPES);

  class_<PointIndices>("PointIndices")
      .smart_ptr_constructor<PointIndices::Ptr>("PointIndices", &std::make_shared<PointIndices>)
      .property("header", &PointIndices::header)
      .property("indices", &PointIndices::indices)
      .smart_ptr<PointIndices::ConstPtr>("PointIndicesConstPtr");

  class_<PCLHeader>("PCLHeader")
      .smart_ptr_constructor<PCLHeader::Ptr>("PCLHeader", &std::make_shared<PCLHeader>)
      .property("seq", &PCLHeader::seq)
      .property("stamp", &PCLHeader::stamp)
      .property("frame_id", &PCLHeader::frame_id)
      .smart_ptr<PCLHeader::ConstPtr>("PCLHeaderConstPtr");

  class_<ModelCoefficients>("ModelCoefficients")
      .smart_ptr_constructor<ModelCoefficients::Ptr>("ModelCoefficients",
                                                     &std::make_shared<ModelCoefficients>)
      .property("header", &ModelCoefficients::header)
      .property("values", &ModelCoefficients::values)
      .smart_ptr<ModelCoefficients::ConstPtr>("ModelCoefficientsConstPtr");

  BOOST_PP_SEQ_FOR_EACH(BIND_toXYZPointCloud, , XYZ_POINT_TYPES);
}