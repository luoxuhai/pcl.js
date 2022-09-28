#include <pcl/point_types.h>
#include <pcl/pcl_config.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/search/kdtree.h>
#include "embind.cpp"

using namespace pcl;

const std::string VERSION = PCL_VERSION_PRETTY;

#define BIND_POINT_CLOUD(PointT)                                                                                   \
  class_<PointCloud<PointT>>("PointCloud" #PointT)                                                                 \
      .smart_ptr_constructor<PointCloud<PointT>::Ptr>("PointCloud" #PointT, &std::make_shared<PointCloud<PointT>>) \
      .property("width", &PointCloud<PointT>::width)                                                               \
      .property("height", &PointCloud<PointT>::height)                                                             \
      .property("points", &PointCloud<PointT>::points)                                                             \
      .property("is_dense", &PointCloud<PointT>::is_dense)                                                         \
      .property("header", &PointCloud<PointT>::header)                                                             \
      .function("isOrganized", &PointCloud<PointT>::isOrganized)                                                   \
      .function("resize", select_overload<void(index_t, const PointT &)>(&PointCloud<PointT>::resize))             \
      .function("push_back", &PointCloud<PointT>::push_back)                                                       \
      .function("clear", &PointCloud<PointT>::clear)                                                               \
      .smart_ptr<PointCloud<PointT>::ConstPtr>("PointCloudConstPtr" #PointT);

#define BIND_POINTS(PointT) register_vector_plus<PointT, Eigen::aligned_allocator<PointT>>("Points" #PointT);

// define PCLBase
#define BIND_PCL_BASE(PointT)                                     \
  class_<PCLBase<PointT>>("PCLBase" #PointT)                      \
      .function("setInputCloud", &PCLBase<PointT>::setInputCloud) \
      .function("getInputCloud", &PCLBase<PointT>::getInputCloud);

double
computeCloudResolution(const PointCloud<PointXYZ>::ConstPtr &cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> squaredDistances(2);
  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
  tree->setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (!std::isfinite((*cloud)[i].x))
      continue;

    // Considering the second neighbor since the first is the point itself.
    nres = tree->nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;

  return resolution;
}

EMSCRIPTEN_BINDINGS(common)
{
  constant("PCL_VERSION", VERSION);

  BIND_POINT_CLOUD(PointXYZ);
  BIND_POINT_CLOUD(PointXYZI);
  BIND_POINT_CLOUD(PointXYZRGB);
  BIND_POINT_CLOUD(PointXYZRGBA);
  BIND_POINT_CLOUD(Normal);
  BIND_POINT_CLOUD(PointNormal);

  BIND_POINTS(PointXYZ);
  BIND_POINTS(PointXYZI);
  BIND_POINTS(PointXYZRGB);
  BIND_POINTS(PointXYZRGBA);
  BIND_POINTS(Normal);
  BIND_POINTS(PointNormal);

  // Bind PCLBase
  BIND_PCL_BASE(PointXYZ);
  BIND_PCL_BASE(PointXYZI);
  BIND_PCL_BASE(PointXYZRGB);
  BIND_PCL_BASE(PointXYZRGBA);
  BIND_PCL_BASE(Normal);
  BIND_PCL_BASE(PointNormal);

  register_vector_plus<float>("VectorFloat");
  register_vector_plus<index_t>("Indices");

  function("computeCloudResolution", &computeCloudResolution);

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
}