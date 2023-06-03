#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <emscripten/val.h>
#include <Eigen/Dense>

#include "embind.hpp"

using namespace pcl;
using namespace emscripten;
thread_local const val Float32Array = emscripten::val::global("Float32Array");


template <typename PointSource,typename PointTarget>
val getFinalTransformation(pcl::Registration<PointSource,PointTarget> &icp) {
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    auto js_result = val::null();
    std::array<float, 16> array;
    int idx = 0;
    for (int col = 0; col < transformation.cols(); ++col) {
        for (int row = 0; row < transformation.rows(); ++row) {
            array[idx++] = transformation(row, col);
        }
    }
    js_result = val(typed_memory_view(array.size(), array.data()));
    return js_result;
}

template <typename PointSource,typename PointTarget>
void align(pcl::IterativeClosestPoint<PointSource,PointTarget> &icp,PointCloud<PointTarget> &aligned,val array) {
    unsigned int length = array["length"].as<float>();
    if (length != 16) {
        icp.align(aligned);
        return;
    }else{
        Eigen::Matrix4f mat;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                mat(i, j) =  array[i * 4 + j].as<float>();
            }
        }
        icp.align(aligned,mat);
        return;
    }
}




#define BIND_Registration(PointSource, PointTarget)                                               \
  class_<pcl::Registration<PointSource, PointTarget>>("Registration" #PointSource #PointTarget)   \
      .function("hasConverged", &pcl::Registration<PointSource, PointTarget>::hasConverged)       \
      .function("getInputSource", &pcl::Registration<PointSource, PointTarget>::getInputSource)   \
      .function("getInputTarget", &pcl::Registration<PointSource, PointTarget>::getInputTarget)   \
      .function("getFitnessScore",                                                                \
                select_overload<double(double)>(                                                  \
                    &pcl::Registration<PointSource, PointTarget>::getFitnessScore))               \
      .function("setMaximumIterations",                                                           \
                &pcl::Registration<PointSource, PointTarget>::setMaximumIterations)               \
      .function("getMaximumIterations",                                                           \
                &pcl::Registration<PointSource, PointTarget>::getMaximumIterations)               \
      .function("setRANSACIterations",                                                            \
                &pcl::Registration<PointSource, PointTarget>::setRANSACIterations)                \
      .function("getRANSACIterations",                                                            \
                &pcl::Registration<PointSource, PointTarget>::getRANSACIterations)                \
      .function("setRANSACOutlierRejectionThreshold",                                             \
                &pcl::Registration<PointSource, PointTarget>::setRANSACOutlierRejectionThreshold) \
      .function("getRANSACOutlierRejectionThreshold",                                             \
                &pcl::Registration<PointSource, PointTarget>::getRANSACOutlierRejectionThreshold) \
      .function("setMaxCorrespondenceDistance",                                                   \
                &pcl::Registration<PointSource, PointTarget>::setMaxCorrespondenceDistance)       \
      .function("getMaxCorrespondenceDistance",                                                   \
                &pcl::Registration<PointSource, PointTarget>::getMaxCorrespondenceDistance)       \
      .function("setTransformationEpsilon",                                                       \
                &pcl::Registration<PointSource, PointTarget>::setTransformationEpsilon)           \
      .function("getTransformationEpsilon",                                                       \
                &pcl::Registration<PointSource, PointTarget>::getTransformationEpsilon)           \
      .function("setTransformationRotationEpsilon",                                               \
                &pcl::Registration<PointSource, PointTarget>::setTransformationRotationEpsilon)   \
      .function("getTransformationRotationEpsilon",                                               \
                &pcl::Registration<PointSource, PointTarget>::getTransformationRotationEpsilon)   \
      .function("setEuclideanFitnessEpsilon",                                                     \
                &pcl::Registration<PointSource, PointTarget>::setEuclideanFitnessEpsilon)         \
      .function("getEuclideanFitnessEpsilon",                                                     \
                &pcl::Registration<PointSource, PointTarget>::getEuclideanFitnessEpsilon)         \
      .function("initCompute", &pcl::Registration<PointSource, PointTarget>::initCompute)         \
      .function("initComputeReciprocal",                                                          \
                &pcl::Registration<PointSource, PointTarget>::initComputeReciprocal)              \
      .function(                                                                                  \
          "align",                                                                                \
          &align<PointSource, PointTarget>)                                                       \
      .function("getFinalTransformation",&getFinalTransformation<PointSource, PointTarget>);

#define BIND_IterativeClosestPoint(PointSource, PointTarget)                                      \
  class_<pcl::IterativeClosestPoint<PointSource, PointTarget>,                                    \
         base<pcl::Registration<PointSource, PointTarget>>>(                                      \
      "IterativeClosestPoint" #PointSource #PointTarget)                                          \
      .constructor()                                                                              \
      .function("setInputSource",                                                                 \
                &pcl::IterativeClosestPoint<PointSource, PointTarget>::setInputSource)            \
      .function("setInputTarget",                                                                 \
                &pcl::IterativeClosestPoint<PointSource, PointTarget>::setInputTarget)            \
      .function(                                                                                  \
          "setUseReciprocalCorrespondences",                                                      \
          &pcl::IterativeClosestPoint<PointSource, PointTarget>::setUseReciprocalCorrespondences) \
      .function(                                                                                  \
          "getUseReciprocalCorrespondences",                                                      \
          &pcl::IterativeClosestPoint<PointSource, PointTarget>::getUseReciprocalCorrespondences);\

#define BIND_SampleConsensusInitialAlignment(PointSource, PointTarget, FeatureT)                   \
  class_<pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>,                 \
         base<pcl::Registration<PointSource, PointTarget>>>(                                       \
      "SampleConsensusInitialAlignment" #PointSource #PointTarget #FeatureT)                       \
      .constructor()                                                                               \
      .function("setInputSource", &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,  \
                                                                        FeatureT>::setInputSource) \
      .function("setInputTarget", &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,  \
                                                                        FeatureT>::setInputTarget) \
      .function("setSourceFeatures",                                                               \
                &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,                    \
                                                      FeatureT>::setSourceFeatures)                \
      .function("setTargetFeatures",                                                               \
                &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,                    \
                                                      FeatureT>::setTargetFeatures)                \
      .function("setMinSampleDistance",                                                            \
                &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,                    \
                                                      FeatureT>::setMinSampleDistance)             \
      .function("setNumberOfSamples",                                                              \
                &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,                    \
                                                      FeatureT>::setNumberOfSamples)               \
      .function("setCorrespondenceRandomness",                                                     \
                &pcl::SampleConsensusInitialAlignment<PointSource, PointTarget,                    \
                                                      FeatureT>::setCorrespondenceRandomness);

EMSCRIPTEN_BINDINGS(registration) {
  BIND_Registration(PointXYZ, PointXYZ);

  BIND_IterativeClosestPoint(PointXYZ, PointXYZ);

  BIND_SampleConsensusInitialAlignment(PointXYZ, PointXYZ, FPFHSignature33);


}
