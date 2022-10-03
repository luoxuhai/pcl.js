#include <pcl/point_types.h>

#include <emscripten/bind.h>

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(point_types) {
  value_object<PointXYZ>("PointXYZ")
      .field("x", &PointXYZ::x)
      .field("y", &PointXYZ::y)
      .field("z", &PointXYZ::z);

  value_object<PointXYZI>("PointXYZI")
      .field("x", &PointXYZI::x)
      .field("y", &PointXYZI::y)
      .field("z", &PointXYZI::z)
      .field("intensity", &PointXYZI::intensity);

  value_object<PointXYZRGB>("PointXYZRGB")
      .field("x", &PointXYZRGB::x)
      .field("y", &PointXYZRGB::y)
      .field("z", &PointXYZRGB::z)
      .field("r", &PointXYZRGB::r)
      .field("g", &PointXYZRGB::g)
      .field("b", &PointXYZRGB::b);

  value_object<PointXYZRGBA>("PointXYZRGBA")
      .field("x", &PointXYZRGBA::x)
      .field("y", &PointXYZRGBA::y)
      .field("z", &PointXYZRGBA::z)
      .field("r", &PointXYZRGBA::r)
      .field("g", &PointXYZRGBA::g)
      .field("b", &PointXYZRGBA::b)
      .field("a", &PointXYZRGBA::a);

  value_object<Normal>("Normal")
      .field("normal_x", &Normal::normal_x)
      .field("normal_y", &Normal::normal_y)
      .field("normal_z", &Normal::normal_z)
      .field("curvature", &Normal::curvature);

  value_object<PointNormal>("PointNormal")
      .field("x", &PointNormal::x)
      .field("y", &PointNormal::y)
      .field("z", &PointNormal::z)
      .field("normal_x", &PointNormal::normal_x)
      .field("normal_y", &PointNormal::normal_y)
      .field("normal_z", &PointNormal::normal_z)
      .field("curvature", &PointNormal::curvature);
}