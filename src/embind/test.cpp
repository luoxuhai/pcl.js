#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>

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

  std::cout << "PCL_VERSION: " << PCL_VERSION_PRETTY << std::endl;
  std::cout << cloud.size() << " data points:" << std::endl;
  for (const auto &point : cloud)
  {
    std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
  }

  return 0;
}