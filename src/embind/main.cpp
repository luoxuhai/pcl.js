#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <emscripten/bind.h>

using namespace emscripten;
using namespace std;

static string VERSION = PCL_VERSION_PRETTY;

int main()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.resize(cloud.width * cloud.height);

  for (auto &point : cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  cout << "PCL_VERSION: " << VERSION << endl;
  cout << cloud.size() << " data points:" << endl;
  for (const auto &point : cloud)
  {
    cout << "    " << point.x << " " << point.y << " " << point.z << endl;
  }

  return 0;
}

EMSCRIPTEN_BINDINGS(main)
{
  constant("PCL_VERSION", VERSION);
}