#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <emscripten/bind.h>
#include "embind.cpp"

// define PassThrough
#define BIND_PASS_THROUGH(PointT)                                                             \
    class_<pcl::PassThrough<PointT>, base<pcl::FilterIndices<PointT>>>("PassThrough" #PointT) \
        .constructor<bool>()                                                                  \
        .function("setFilterFieldName", &pcl::PassThrough<PointT>::setFilterFieldName)        \
        .function("getFilterFieldName", &pcl::PassThrough<PointT>::getFilterFieldName)        \
        .function("setFilterLimits", &pcl::PassThrough<PointT>::setFilterLimits)              \
        .function("getFilterLimits", &getFilterLimits<PointT>);

// define FilterIndices
#define BIND_FILTER_INDICES(PointT)                                                        \
    class_<pcl::FilterIndices<PointT>, base<pcl::Filter<PointT>>>("FilterIndices" #PointT) \
        .function("setNegative", &pcl::FilterIndices<PointT>::setNegative)                 \
        .function("getNegative", &pcl::FilterIndices<PointT>::getNegative)                 \
        .function("setKeepOrganized", &pcl::FilterIndices<PointT>::setKeepOrganized)       \
        .function("getKeepOrganized", &pcl::FilterIndices<PointT>::getKeepOrganized)       \
        .function("setUserFilterValue", &pcl::FilterIndices<PointT>::setUserFilterValue);

// define Filter
#define BIND_FILTER(PointT)                                                   \
    class_<pcl::Filter<PointT>, base<pcl::PCLBase<PointT>>>("Filter" #PointT) \
        .function("getRemovedIndices", &getRemovedIndices<PointT>)            \
        .function("filter", &filter<PointT>);

// define PCLBase
#define BIND_PCL_BASE(PointT)                                            \
    class_<pcl::PCLBase<PointT>>("PCLBase" #PointT)                      \
        .function("setInputCloud", &pcl::PCLBase<PointT>::setInputCloud) \
        .function("getInputCloud", &pcl::PCLBase<PointT>::getInputCloud);

// define VoxelGrid
#define BIND_VOXEL_GRID(PointT)                                                                              \
    class_<pcl::VoxelGrid<PointT>, base<pcl::Filter<PointT>>>("VoxelGrid" #PointT)                           \
        .constructor()                                                                                       \
        .function("setLeafSize",                                                                             \
                  select_overload<void(float, float, float)>(&pcl::VoxelGrid<PointT>::setLeafSize))          \
        .function("setDownsampleAllData", &pcl::VoxelGrid<PointT>::setDownsampleAllData)                     \
        .function("getDownsampleAllData", &pcl::VoxelGrid<PointT>::getDownsampleAllData)                     \
        .function("setMinimumPointsNumberPerVoxel", &pcl::VoxelGrid<PointT>::setMinimumPointsNumberPerVoxel) \
        .function("getMinimumPointsNumberPerVoxel", &pcl::VoxelGrid<PointT>::getMinimumPointsNumberPerVoxel);

// define StatisticalOutlierRemoval
#define BIND_SOR(PointT)                                                                                                  \
    class_<pcl::StatisticalOutlierRemoval<PointT>, base<pcl::FilterIndices<PointT>>>("StatisticalOutlierRemoval" #PointT) \
        .constructor<bool>()                                                                                              \
        .function("setMeanK", &pcl::StatisticalOutlierRemoval<PointT>::setMeanK)                                          \
        .function("getMeanK", &pcl::StatisticalOutlierRemoval<PointT>::getMeanK)                                          \
        .function("setStddevMulThresh", &pcl::StatisticalOutlierRemoval<PointT>::setStddevMulThresh)                      \
        .function("getStddevMulThresh", &pcl::StatisticalOutlierRemoval<PointT>::getStddevMulThresh);

// define RadiusOutlierRemoval
#define BIND_ROR(PointT)                                                                                        \
    class_<pcl::RadiusOutlierRemoval<PointT>, base<pcl::FilterIndices<PointT>>>("RadiusOutlierRemoval" #PointT) \
        .constructor<bool>()                                                                                    \
        .function("setRadiusSearch", &pcl::RadiusOutlierRemoval<PointT>::setRadiusSearch)                       \
        .function("getRadiusSearch", &pcl::RadiusOutlierRemoval<PointT>::getRadiusSearch)                       \
        .function("setMinNeighborsInRadius", &pcl::RadiusOutlierRemoval<PointT>::setMinNeighborsInRadius)       \
        .function("getMinNeighborsInRadius", &pcl::RadiusOutlierRemoval<PointT>::getMinNeighborsInRadius);

struct FilterLimits
{
    float min;
    float max;
};

template <typename PointT>
FilterLimits getFilterLimits(pcl::PassThrough<PointT> &passThrough)
{
    float limit_min;
    float limit_max;
    passThrough.getFilterLimits(limit_min, limit_max);

    FilterLimits limits = {.min = limit_min, .max = limit_max};

    return limits;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filter(pcl::Filter<PointT> &filter, typename pcl::PointCloud<PointT>::Ptr &output)
{
    if (output == nullptr)
    {
        pcl::PointCloud<PointT> cloud;
        filter.filter(cloud);
        return cloud.makeShared();
    }
    else
    {
        filter.filter(*output);
        return output;
    }
}

template <typename PointT>
pcl::Indices getRemovedIndices(pcl::Filter<PointT> &filter)
{
    pcl::PointIndices pointIndices;
    filter.getRemovedIndices(pointIndices);
    return pointIndices.indices;
}

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(filters)
{
    // Bind PassThrough
    BIND_PASS_THROUGH(PointXYZ);
    BIND_PASS_THROUGH(PointXYZI);
    BIND_PASS_THROUGH(PointXYZRGB);
    BIND_PASS_THROUGH(PointXYZRGBA);
    BIND_PASS_THROUGH(PointNormal);

    // Bind FilterIndices
    BIND_FILTER_INDICES(PointXYZ);
    BIND_FILTER_INDICES(PointXYZI);
    BIND_FILTER_INDICES(PointXYZRGB);
    BIND_FILTER_INDICES(PointXYZRGBA);
    BIND_FILTER_INDICES(Normal);
    BIND_FILTER_INDICES(PointNormal);

    // Bind Filter
    BIND_FILTER(PointXYZ);
    BIND_FILTER(PointXYZI);
    BIND_FILTER(PointXYZRGB);
    BIND_FILTER(PointXYZRGBA);
    BIND_FILTER(Normal);
    BIND_FILTER(PointNormal);

    // Bind PCLBase
    BIND_PCL_BASE(PointXYZ);
    BIND_PCL_BASE(PointXYZI);
    BIND_PCL_BASE(PointXYZRGB);
    BIND_PCL_BASE(PointXYZRGBA);
    BIND_PCL_BASE(Normal);
    BIND_PCL_BASE(PointNormal);

    // Bind VoxelGrid
    BIND_VOXEL_GRID(PointXYZ);
    BIND_VOXEL_GRID(PointXYZI);
    BIND_VOXEL_GRID(PointXYZRGB);
    BIND_VOXEL_GRID(PointXYZRGBA);
    BIND_VOXEL_GRID(PointNormal);

    // Bind StatisticalOutlierRemoval
    BIND_SOR(PointXYZ);
    BIND_SOR(PointXYZI);
    BIND_SOR(PointXYZRGB);
    BIND_SOR(PointXYZRGBA);
    BIND_SOR(PointNormal);

    // Bind RadiusOutlierRemoval
    BIND_ROR(PointXYZ);
    BIND_ROR(PointXYZI);
    BIND_ROR(PointXYZRGB);
    BIND_ROR(PointXYZRGBA);
    BIND_ROR(PointNormal);

    value_array<FilterLimits>("FilterLimits")
        .element(&FilterLimits::min)
        .element(&FilterLimits::max);

    register_vector_plus<index_t>("Indices");
}
