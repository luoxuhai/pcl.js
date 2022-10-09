#ifndef BIND_IO_PCD_DATA_READER_IMPL_H_
#define BIND_IO_PCD_DATA_READER_IMPL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <typename PointT>
void readBodyASCII(const pcl::PointCloud<PointT> &cloud, const int precision) {}

template <typename PointT>
void readBodyBinary(const pcl::PointCloud<PointT> &cloud, const int precision) {}

template <typename PointT>
void writeBinaryCompressed(const pcl::PointCloud<PointT> &cloud, const int precision) {}

template <typename PointT>
int read(const std::string &file_name, pcl::PCLPointCloud2 &cloud, Eigen::Vector4f &origin,
         Eigen::Quaternionf &orientation, int &pcd_version, const int offset) {
  pcl::console::TicToc tt;
  tt.tic();

  if (file_name.empty() || !boost::filesystem::exists(file_name)) {
    PCL_ERROR("[pcl::PCDReader::read] Could not find file '%s'.\n", file_name.c_str());
    return (-1);
  }

  int data_type;
  unsigned int data_idx;

  int res =
      readHeader(file_name, cloud, origin, orientation, pcd_version, data_type, data_idx, offset);

  if (res < 0) return (res);

  // if ascii
  if (data_type == 0) {
    // Re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open(file_name.c_str());
    if (!fs.is_open() || fs.fail()) {
      PCL_ERROR("[pcl::PCDReader::read] Could not open file %s.\n", file_name.c_str());
      return (-1);
    }

    fs.seekg(data_idx + offset);

    // Read the rest of the file
    res = readBodyASCII(fs, cloud, pcd_version);

    // Close file
    fs.close();
  } else
  /// ---[ Binary mode only
  /// We must re-open the file and read with mmap () for binary
  {
    // Open for reading
    int fd = io::raw_open(file_name.c_str(), O_RDONLY);
    if (fd == -1) {
      PCL_ERROR("[pcl::PCDReader::read] Failure to open file %s\n", file_name.c_str());
      return (-1);
    }

    // Infer file size
    const std::size_t file_size = io::raw_lseek(fd, 0, SEEK_END);
    io::raw_lseek(fd, 0, SEEK_SET);

    std::size_t mmap_size = offset + data_idx;  // ...because we mmap from the start of the file.
    if (data_type == 2) {
      // Seek to real start of data.
      long result = io::raw_lseek(fd, offset + data_idx, SEEK_SET);
      if (result < 0) {
        io::raw_close(fd);
        PCL_ERROR("[pcl::PCDReader::read] lseek errno: %d strerror: %s\n", errno, strerror(errno));
        PCL_ERROR("[pcl::PCDReader::read] Error during lseek ()!\n");
        return (-1);
      }

      // Read compressed size to compute how much must be mapped
      unsigned int compressed_size = 0;
      ssize_t num_read = io::raw_read(fd, &compressed_size, 4);
      if (num_read < 0) {
        io::raw_close(fd);
        PCL_ERROR("[pcl::PCDReader::read] read errno: %d strerror: %s\n", errno, strerror(errno));
        PCL_ERROR("[pcl::PCDReader::read] Error during read()!\n");
        return (-1);
      }
      mmap_size += compressed_size;
      // Add the 8 bytes used to store the compressed and uncompressed size
      mmap_size += 8;

      // Reset position
      io::raw_lseek(fd, 0, SEEK_SET);
    } else {
      mmap_size += cloud.data.size();
    }

    if (mmap_size > file_size) {
      io::raw_close(fd);
      PCL_ERROR("[pcl::PCDReader::read] Corrupted PCD file. The file is smaller than expected!\n");
      return (-1);
    }

    // Prepare the map
#ifdef _WIN32
    // As we don't know the real size of data (compressed or not),
    // we set dwMaximumSizeHigh = dwMaximumSizeLow = 0 so as to map the whole file
    HANDLE fm = CreateFileMapping((HANDLE)_get_osfhandle(fd), NULL, PAGE_READONLY, 0, 0, NULL);
    // As we don't know the real size of data (compressed or not),
    // we set dwNumberOfBytesToMap = 0 so as to map the whole file
    unsigned char *map = static_cast<unsigned char *>(MapViewOfFile(fm, FILE_MAP_READ, 0, 0, 0));
    if (map == NULL) {
      CloseHandle(fm);
      io::raw_close(fd);
      PCL_ERROR("[pcl::PCDReader::read] Error mapping view of file, %s\n", file_name.c_str());
      return (-1);
    }
#else
    unsigned char *map =
        static_cast<unsigned char *>(::mmap(nullptr, mmap_size, PROT_READ, MAP_SHARED, fd, 0));
    if (map == reinterpret_cast<unsigned char *>(-1))  // MAP_FAILED
    {
      io::raw_close(fd);
      PCL_ERROR("[pcl::PCDReader::read] Error preparing mmap for binary PCD file.\n");
      return (-1);
    }
#endif

    res = readBodyBinary(map, cloud, pcd_version, data_type == 2, offset + data_idx);

    // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile(map);
    CloseHandle(fm);
#else
    if (::munmap(map, mmap_size) == -1) {
      io::raw_close(fd);
      PCL_ERROR("[pcl::PCDReader::read] Munmap failure\n");
      return (-1);
    }
#endif
    io::raw_close(fd);
  }
  double total_time = tt.toc();
  PCL_DEBUG(
      "[pcl::PCDReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available "
      "dimensions: %s.\n",
      file_name.c_str(), cloud.is_dense ? "dense" : "non-dense", total_time,
      cloud.width * cloud.height, pcl::getFieldsList(cloud).c_str());
  return res;
}

#endif /* BIND_IO_PCD_DATA_READER_IMPL_H_ */