#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <emscripten/bind.h>
// #include <fstream>

using namespace emscripten;

static char VERSION[] = "1.12.1";

auto runTest(int width = 5)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width = width;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.resize(cloud.width * cloud.height);

  for (auto &point : cloud)
  {
    point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  std::cout << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

  for (const auto &point : cloud)
    std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  // std::ofstream out("out.txt");
  // if (out.is_open())
  // {
  //   out << "This is a line.\n";
  //   out << "This is another line.\n";
  //   out.close();
  // }

  pcl::io::savePCDFile("./test_pcd.pcd", cloud);
  return 0;
}

int main()
{
  std::cout << "init" << std::endl;
  return 0;
}

int savePCDFile(const std::string &file_name, const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  return pcl::io::savePCDFile(file_name, cloud);
}

EMSCRIPTEN_BINDINGS(io)
{
  function("runTest", &runTest);
  function("io.savePCDFile", savePCDFile);
}