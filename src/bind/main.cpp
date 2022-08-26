#include <iostream>
#include <pcl/pcl_config.h>
#include <emscripten/bind.h>

using namespace emscripten;

const std::string VERSION = PCL_VERSION_PRETTY;

int main()
{
  std::cout << "PCL_VERSION: " << VERSION << std::endl;
  return 0;
}

EMSCRIPTEN_BINDINGS(main)
{
  constant("PCL_VERSION", VERSION);
}