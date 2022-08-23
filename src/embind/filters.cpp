#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <emscripten/bind.h>

typedef pcl::PointXYZ PointCloudXYZ;

using namespace emscripten;

EMSCRIPTEN_BINDINGS(Filter)
{
    class_<pcl::PassThrough<PointCloudXYZ>, base<pcl::FilterIndices<PointCloudXYZ>>>("PassThrough")
        .constructor<bool>()
        .function("setFilterFieldName", &pcl::PassThrough<PointCloudXYZ>::setFilterFieldName)
        .function("getFilterFieldName", &pcl::PassThrough<PointCloudXYZ>::getFilterFieldName)
        .function("setFilterLimits", &pcl::PassThrough<PointCloudXYZ>::setFilterLimits)
        .function("getFilterLimits",
                  reinterpret_cast<void (pcl::PassThrough<PointCloudXYZ>::*)(const float &, const float &) const>(&pcl::PassThrough<PointCloudXYZ>::getFilterLimits));
    // TODO: getFilterLimits
    // .function("getFilterLimits", &pcl::PassThrough<PointCloudXYZ>::setFilterLimits);

    class_<pcl::FilterIndices<PointCloudXYZ>, base<pcl::Filter<PointCloudXYZ>>>("FilterIndices")
        .function("setNegative", &pcl::FilterIndices<PointCloudXYZ>::setNegative)
        .function("getNegative", &pcl::FilterIndices<PointCloudXYZ>::getNegative)
        .function("setKeepOrganized", &pcl::FilterIndices<PointCloudXYZ>::setKeepOrganized)
        .function("getKeepOrganized", &pcl::FilterIndices<PointCloudXYZ>::getKeepOrganized)
        .function("setUserFilterValue", &pcl::FilterIndices<PointCloudXYZ>::setUserFilterValue);

    class_<pcl::Filter<PointCloudXYZ>, base<pcl::PCLBase<PointCloudXYZ>>>("Filter")
        .function("filter", &pcl::Filter<PointCloudXYZ>::filter);

    class_<pcl::PCLBase<PointCloudXYZ>>("PCLBase")
        .function("setInputCloud", &pcl::PCLBase<PointCloudXYZ>::setInputCloud)
        .function("getInputCloud", &pcl::PCLBase<PointCloudXYZ>::getInputCloud);
}
