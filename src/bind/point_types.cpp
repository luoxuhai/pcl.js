#include <pcl/point_types.h>

#include <emscripten/bind.h>

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(point_types) {
  value_object<PointXY>("PointXY").field("x", &PointXY::x).field("y", &PointXY::y);

  value_object<PointXYZ>("PointXYZ")
      .field("x", &PointXYZ::x)
      .field("y", &PointXYZ::y)
      .field("z", &PointXYZ::z);

  value_object<PointXYZI>("PointXYZI")
      .field("x", &PointXYZI::x)
      .field("y", &PointXYZI::y)
      .field("z", &PointXYZI::z)
      .field("intensity", &PointXYZI::intensity);

  value_object<InterestPoint>("InterestPoint")
      .field("x", &InterestPoint::x)
      .field("y", &InterestPoint::y)
      .field("z", &InterestPoint::z)
      .field("strength", &InterestPoint::strength);

  value_object<PointXYZL>("PointXYZL")
      .field("x", &PointXYZL::x)
      .field("y", &PointXYZL::y)
      .field("z", &PointXYZL::z)
      .field("label", &PointXYZL::label);

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

  value_object<PointXYZRGBL>("PointXYZRGBL")
      .field("x", &PointXYZRGBL::x)
      .field("y", &PointXYZRGBL::y)
      .field("z", &PointXYZRGBL::z)
      .field("r", &PointXYZRGBL::r)
      .field("g", &PointXYZRGBL::g)
      .field("b", &PointXYZRGBL::b)
      .field("label", &PointXYZRGBL::label);

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

  value_object<PointXYZLNormal>("PointXYZLNormal")
      .field("x", &PointXYZLNormal::x)
      .field("y", &PointXYZLNormal::y)
      .field("z", &PointXYZLNormal::z)
      .field("label", &PointXYZLNormal::label)
      .field("normal_x", &PointXYZLNormal::normal_x)
      .field("normal_y", &PointXYZLNormal::normal_y)
      .field("normal_z", &PointXYZLNormal::normal_z)
      .field("curvature", &PointXYZLNormal::curvature);

  value_object<PointXYZINormal>("PointXYZINormal")
      .field("x", &PointXYZINormal::x)
      .field("y", &PointXYZINormal::y)
      .field("z", &PointXYZINormal::z)
      .field("intensity", &PointXYZINormal::intensity)
      .field("normal_x", &PointXYZINormal::normal_x)
      .field("normal_y", &PointXYZINormal::normal_y)
      .field("normal_z", &PointXYZINormal::normal_z)
      .field("curvature", &PointXYZINormal::curvature);

  value_object<PointXYZRGBNormal>("PointXYZRGBNormal")
      .field("x", &PointXYZRGBNormal::x)
      .field("y", &PointXYZRGBNormal::y)
      .field("z", &PointXYZRGBNormal::z)
      .field("r", &PointXYZRGBNormal::r)
      .field("g", &PointXYZRGBNormal::g)
      .field("b", &PointXYZRGBNormal::b)
      .field("normal_x", &PointXYZRGBNormal::normal_x)
      .field("normal_y", &PointXYZRGBNormal::normal_y)
      .field("normal_z", &PointXYZRGBNormal::normal_z)
      .field("curvature", &PointXYZRGBNormal::curvature);

  value_object<PointSurfel>("PointSurfel")
      .field("x", &PointSurfel::x)
      .field("y", &PointSurfel::y)
      .field("z", &PointSurfel::z)
      .field("normal_x", &PointSurfel::normal_x)
      .field("normal_y", &PointSurfel::normal_y)
      .field("normal_z", &PointSurfel::normal_z)
      .field("r", &PointSurfel::r)
      .field("g", &PointSurfel::g)
      .field("b", &PointSurfel::b)
      .field("a", &PointSurfel::a)
      .field("radius", &PointSurfel::radius)
      .field("confidence", &PointSurfel::confidence)
      .field("curvature", &PointSurfel::curvature);
}