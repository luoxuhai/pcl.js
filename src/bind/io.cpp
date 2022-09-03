#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <emscripten/bind.h>

using namespace pcl;

template <typename PointT>
typename PointCloud<PointT>::Ptr loadPCDFile(const std::string &file_name)
{
  typename PointCloud<PointT>::Ptr cloud_ptr(new PointCloud<PointT>);
  int flag = io::loadPCDFile(file_name, *cloud_ptr);
  return cloud_ptr;
}

std::vector<pcl::PCLPointField> readPCDHeader(const std::string &file_name)
{
  pcl::PCLPointCloud2 cloud;
  pcl::PCDReader reader;
  reader.readHeader(file_name, cloud);
  return cloud.fields;
}

#define BIND_LOAD_PCD_FILE(PointT) function("loadPCDFile" #PointT, &loadPCDFile<PointT>);
#define BIND_SAVE_PCD_FILE(PointT) function("savePCDFile" #PointT, select_overload<int(const std::string &, const PointCloud<PointT> &, bool)>(&io::savePCDFile));
#define BIND_SAVE_PCD_FBC(PointT) function("savePCDFileBinaryCompressed" #PointT, &io::savePCDFileBinaryCompressed<PointT>);

using namespace emscripten;

EMSCRIPTEN_BINDINGS(io)
{
  BIND_LOAD_PCD_FILE(PointXYZ);
  BIND_LOAD_PCD_FILE(PointXYZI);
  BIND_LOAD_PCD_FILE(PointXYZRGB);
  BIND_LOAD_PCD_FILE(PointXYZRGBA);
  BIND_LOAD_PCD_FILE(Normal);
  BIND_LOAD_PCD_FILE(PointNormal);

  BIND_SAVE_PCD_FILE(PointXYZ);
  BIND_SAVE_PCD_FILE(PointXYZI);
  BIND_SAVE_PCD_FILE(PointXYZRGB);
  BIND_SAVE_PCD_FILE(PointXYZRGBA);
  BIND_SAVE_PCD_FILE(Normal);
  BIND_SAVE_PCD_FILE(PointNormal);

  BIND_SAVE_PCD_FBC(PointXYZ);
  BIND_SAVE_PCD_FBC(PointXYZI);
  BIND_SAVE_PCD_FBC(PointXYZRGB);
  BIND_SAVE_PCD_FBC(PointXYZRGBA);
  BIND_SAVE_PCD_FBC(Normal);
  BIND_SAVE_PCD_FBC(PointNormal);

  function("readPCDHeader", &readPCDHeader);

  register_vector<pcl::PCLPointField>("PCLPointFields");

  value_object<pcl::PCLPointField>("PCLPointField")
      .field("name", &pcl::PCLPointField::name)
      .field("datatype", &pcl::PCLPointField::datatype);
}