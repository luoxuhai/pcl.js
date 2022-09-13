#include <iostream>
#include <pcl/PointIndices.h>
#include <emscripten/bind.h>

using namespace pcl;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(point_indices) 
{ 
  value_object<PointIndices>("PointIndices")
    .field("indices", &PointIndices::indices);
}