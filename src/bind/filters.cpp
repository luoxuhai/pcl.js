#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <emscripten/bind.h>

typedef pcl::PointXYZ PointCloudXYZ;

using namespace emscripten;

struct FilterLimits
{
    float min;
    float max;
};

FilterLimits getFilterLimits(pcl::PassThrough<PointCloudXYZ> &passThrough)
{
    float limit_min;
    float limit_max;
    passThrough.getFilterLimits(limit_min, limit_max);

    FilterLimits limits = {.min = limit_min, .max = limit_max};

    return limits;
}

pcl::PointCloud<PointCloudXYZ> filter(pcl::Filter<PointCloudXYZ> &filter)
{
    pcl::PointCloud<PointCloudXYZ> cloud;
    filter.filter(cloud);

    return cloud;
}

EMSCRIPTEN_BINDINGS(filters)
{
    // PassThrough

    class_<pcl::PassThrough<PointCloudXYZ>, base<pcl::FilterIndices<PointCloudXYZ>>>("PassThrough")
        .constructor<bool>()
        .function("setFilterFieldName", &pcl::PassThrough<PointCloudXYZ>::setFilterFieldName)
        .function("getFilterFieldName", &pcl::PassThrough<PointCloudXYZ>::getFilterFieldName)
        .function("setFilterLimits", &pcl::PassThrough<PointCloudXYZ>::setFilterLimits)
        .function("getFilterLimits", &getFilterLimits);

    class_<pcl::FilterIndices<PointCloudXYZ>, base<pcl::Filter<PointCloudXYZ>>>("FilterIndices")
        .function("setNegative", &pcl::FilterIndices<PointCloudXYZ>::setNegative)
        .function("getNegative", &pcl::FilterIndices<PointCloudXYZ>::getNegative)
        .function("setKeepOrganized", &pcl::FilterIndices<PointCloudXYZ>::setKeepOrganized)
        .function("getKeepOrganized", &pcl::FilterIndices<PointCloudXYZ>::getKeepOrganized)
        .function("setUserFilterValue", &pcl::FilterIndices<PointCloudXYZ>::setUserFilterValue);

    class_<pcl::Filter<PointCloudXYZ>, base<pcl::PCLBase<PointCloudXYZ>>>("Filter")
        .function("filter", &filter);

    class_<pcl::PCLBase<PointCloudXYZ>>("PCLBase")
        .function("setInputCloud", &pcl::PCLBase<PointCloudXYZ>::setInputCloud)
        .function("getInputCloud", &pcl::PCLBase<PointCloudXYZ>::getInputCloud);

    // VoxelGrid

    class_<pcl::VoxelGrid<PointCloudXYZ>, base<pcl::Filter<PointCloudXYZ>>>("VoxelGrid")
        .constructor()
        .function("setLeafSize",
                  select_overload<void(float, float, float)>(&pcl::VoxelGrid<PointCloudXYZ>::setLeafSize));

    // StatisticalOutlierRemoval

    class_<pcl::StatisticalOutlierRemoval<PointCloudXYZ>, base<pcl::FilterIndices<PointCloudXYZ>>>("StatisticalOutlierRemoval")
        .constructor<bool>()
        .function("setMeanK", &pcl::StatisticalOutlierRemoval<PointCloudXYZ>::setMeanK)
        .function("getMeanK", &pcl::StatisticalOutlierRemoval<PointCloudXYZ>::getMeanK)
        .function("setStddevMulThresh", &pcl::StatisticalOutlierRemoval<PointCloudXYZ>::setStddevMulThresh)
        .function("getStddevMulThresh", &pcl::StatisticalOutlierRemoval<PointCloudXYZ>::getStddevMulThresh);

    // RadiusOutlierRemoval

    class_<pcl::RadiusOutlierRemoval<PointCloudXYZ>, base<pcl::FilterIndices<PointCloudXYZ>>>("RadiusOutlierRemoval")
        .constructor<bool>()
        .function("setRadiusSearch", &pcl::RadiusOutlierRemoval<PointCloudXYZ>::setRadiusSearch)
        .function("getRadiusSearch", &pcl::RadiusOutlierRemoval<PointCloudXYZ>::getRadiusSearch)
        .function("setMinNeighborsInRadius", &pcl::RadiusOutlierRemoval<PointCloudXYZ>::setMinNeighborsInRadius)
        .function("getMinNeighborsInRadius", &pcl::RadiusOutlierRemoval<PointCloudXYZ>::getMinNeighborsInRadius);

    value_array<FilterLimits>("FilterLimits")
        .element(&FilterLimits::min)
        .element(&FilterLimits::max);
}
