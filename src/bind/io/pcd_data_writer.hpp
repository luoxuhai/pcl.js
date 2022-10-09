#ifndef BIND_IO_PCD_DATA_WRITER_IMPL_H_
#define BIND_IO_PCD_DATA_WRITER_IMPL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <typename PointT>
std::string generateHeader(const pcl::PointCloud<PointT> &cloud,
                           const int nr_points = std::numeric_limits<int>::max()) {
  std::ostringstream oss;
  oss.imbue(std::locale::classic());

  oss << "# .PCD v0.7 - Point Cloud Data file format"
         "\nVERSION 0.7"
         "\nFIELDS";

  const auto fields = pcl::getFields<PointT>();

  std::stringstream field_names, field_types, field_sizes, field_counts;
  for (const auto &field : fields) {
    if (field.name == "_") continue;
    // Add the regular dimension
    field_names << " " << field.name;
    field_sizes << " " << pcl::getFieldSize(field.datatype);
    if ("rgb" == field.name)
      field_types << " "
                  << "U";
    else
      field_types << " " << pcl::getFieldType(field.datatype);
    int count = std::abs(static_cast<int>(field.count));
    if (count == 0) count = 1;  // check for 0 counts (coming from older converter code)
    field_counts << " " << count;
  }
  oss << field_names.str();
  oss << "\nSIZE" << field_sizes.str() << "\nTYPE" << field_types.str() << "\nCOUNT"
      << field_counts.str();
  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max())
    oss << "\nWIDTH " << nr_points << "\nHEIGHT " << 1 << "\n";
  else
    oss << "\nWIDTH " << cloud.width << "\nHEIGHT " << cloud.height << "\n";

  oss << "VIEWPOINT " << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " "
      << cloud.sensor_origin_[2] << " " << cloud.sensor_orientation_.w() << " "
      << cloud.sensor_orientation_.x() << " " << cloud.sensor_orientation_.y() << " "
      << cloud.sensor_orientation_.z() << "\n";

  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max())
    oss << "POINTS " << nr_points << "\n";
  else
    oss << "POINTS " << cloud.size() << "\n";

  return (oss.str());
}

template <typename PointT>
const char *writeASCII(const pcl::PointCloud<PointT> &cloud, std::size_t &data_size,
                       const int precision = 8) {
  const auto fields = pcl::getFields<PointT>();

  std::ostringstream stream;
  stream.precision(precision);
  stream.imbue(std::locale::classic());
  // Write the header information
  stream << generateHeader(cloud) << "DATA ascii\n";
  // Iterate through the points
  for (const auto &point : cloud) {
    for (std::size_t d = 0; d < fields.size(); ++d) {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (fields[d].name == "_") continue;

      int count = fields[d].count;
      if (count == 0)
        count = 1;  // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c) {
        switch (fields[d].datatype) {
          case pcl::PCLPointField::INT8: {
            std::int8_t value;
            memcpy(
                &value,
                reinterpret_cast<const char *>(&point) + fields[d].offset + c * sizeof(std::int8_t),
                sizeof(std::int8_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT8: {
            std::uint8_t value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset +
                       c * sizeof(std::uint8_t),
                   sizeof(std::uint8_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT16: {
            std::int16_t value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset +
                       c * sizeof(std::int16_t),
                   sizeof(std::int16_t));
            stream << boost::numeric_cast<std::int16_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT16: {
            std::uint16_t value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset +
                       c * sizeof(std::uint16_t),
                   sizeof(std::uint16_t));
            stream << boost::numeric_cast<std::uint16_t>(value);
            break;
          }
          case pcl::PCLPointField::INT32: {
            std::int32_t value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset +
                       c * sizeof(std::int32_t),
                   sizeof(std::int32_t));
            stream << boost::numeric_cast<std::int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT32: {
            std::uint32_t value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset +
                       c * sizeof(std::uint32_t),
                   sizeof(std::uint32_t));
            stream << boost::numeric_cast<std::uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT32: {
            /*
             * Despite the float type, store the rgb field as uint32
             * because several fully opaque color values are mapped to
             * nan.
             */
            if ("rgb" == fields[d].name) {
              std::uint32_t value;
              memcpy(&value,
                     reinterpret_cast<const char *>(&point) + fields[d].offset + c * sizeof(float),
                     sizeof(float));
              stream << boost::numeric_cast<std::uint32_t>(value);
              break;
            }
            float value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset + c * sizeof(float),
                   sizeof(float));
            if (std::isnan(value))
              stream << "nan";
            else
              stream << boost::numeric_cast<float>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT64: {
            double value;
            memcpy(&value,
                   reinterpret_cast<const char *>(&point) + fields[d].offset + c * sizeof(double),
                   sizeof(double));
            if (std::isnan(value))
              stream << "nan";
            else
              stream << boost::numeric_cast<double>(value);
            break;
          }
          default:
            PCL_WARN("[pcl::PCDWriter::writeASCII] Incorrect field data type specified (%d)!\n",
                     fields[d].datatype);
            break;
        }

        if (d < fields.size() - 1 || c < static_cast<int>(fields[d].count - 1)) stream << " ";
      }
    }
    stream << "\n";
  }

  std::string str = stream.str();
  data_size = str.length();
  return str.c_str();
}

template <typename PointT>
const char *writeBinary(const pcl::PointCloud<PointT> &cloud, std::size_t &data_size) {
  std::string header = generateHeader<PointT>(cloud) + "DATA binary\n";
  std::size_t data_idx = header.length();
  auto fields = pcl::getFields<PointT>();
  std::vector<int> fields_sizes;
  std::size_t fsize = 0;
  std::size_t nri = 0;

  // Compute the total size of the fields
  for (const auto &field : fields) {
    if (field.name == "_") continue;

    int fs = field.count * getFieldSize(field.datatype);
    fsize += fs;
    fields_sizes.push_back(fs);
    fields[nri++] = field;
  }
  fields.resize(nri);

  data_size = cloud.size() * fsize + data_idx;
  char *data = (char *)malloc(sizeof(char) * data_size);

  // Copy the header
  memcpy(&data[0], header.c_str(), data_idx);

  // Copy the data
  char *out = &data[0] + data_idx;
  for (const auto &point : cloud) {
    int nrj = 0;
    for (const auto &field : fields) {
      memcpy(out, reinterpret_cast<const char *>(&point) + field.offset, fields_sizes[nrj]);
      out += fields_sizes[nrj++];
    }
  }

  return data;
}

template <typename PointT>
const char *writeBinaryCompressed(const pcl::PointCloud<PointT> &cloud, std::size_t &data_size) {
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeader<PointT>(cloud) << "DATA binary_compressed\n";
  oss.flush();
  data_idx = static_cast<int>(oss.tellp());
  auto fields = pcl::getFields<PointT>();
  std::size_t fsize = 0;
  std::size_t data_size_ = 0;
  std::size_t nri = 0;
  std::vector<int> fields_sizes(fields.size());
  // Compute the total size of the fields
  for (const auto &field : fields) {
    if (field.name == "_") continue;

    fields_sizes[nri] = field.count * pcl::getFieldSize(field.datatype);
    fsize += fields_sizes[nri];
    fields[nri] = field;
    ++nri;
  }
  fields_sizes.resize(nri);
  fields.resize(nri);

  // Compute the size of data
  data_size_ = cloud.size() * fsize;

  //////////////////////////////////////////////////////////////////////
  // Empty array holding only the valid data
  // data_size_ = nr_points * point_size
  //           = nr_points * (sizeof_field_1 + sizeof_field_2 + ... sizeof_field_n)
  //           = sizeof_field_1 * nr_points + sizeof_field_2 * nr_points + ... sizeof_field_n *
  //           nr_points
  char *only_valid_data = static_cast<char *>(malloc(data_size_));

  // Convert the XYZRGBXYZRGB structure to XXYYZZRGBRGB to aid compression. For
  // this, we need a vector of fields.size () (4 in this case), which points to
  // each individual plane:
  //   pters[0] = &only_valid_data[offset_of_plane_x];
  //   pters[1] = &only_valid_data[offset_of_plane_y];
  //   pters[2] = &only_valid_data[offset_of_plane_z];
  //   pters[3] = &only_valid_data[offset_of_plane_RGB];
  //
  std::vector<char *> pters(fields.size());
  std::size_t toff = 0;
  for (std::size_t i = 0; i < pters.size(); ++i) {
    pters[i] = &only_valid_data[toff];
    toff += static_cast<std::size_t>(fields_sizes[i]) * cloud.size();
  }

  // Go over all the points, and copy the data in the appropriate places
  for (const auto &point : cloud) {
    for (std::size_t j = 0; j < fields.size(); ++j) {
      memcpy(pters[j], reinterpret_cast<const char *>(&point) + fields[j].offset, fields_sizes[j]);
      // Increment the pointer
      pters[j] += fields_sizes[j];
    }
  }

  char *temp_buf = static_cast<char *>(
      malloc(static_cast<std::size_t>(static_cast<float>(data_size_) * 1.5f + 8.0f)));
  // Compress the valid data
  unsigned int compressed_size =
      pcl::lzfCompress(only_valid_data, static_cast<std::uint32_t>(data_size_), &temp_buf[8],
                       static_cast<std::uint32_t>(static_cast<float>(data_size_) * 1.5f));
  unsigned int compressed_final_size = 0;
  // Was the compression successful?
  if (compressed_size) {
    char *header = &temp_buf[0];
    memcpy(&header[0], &compressed_size, sizeof(unsigned int));
    memcpy(&header[4], &data_size_, sizeof(unsigned int));
    data_size_ = compressed_size + 8;
    compressed_final_size = static_cast<std::uint32_t>(data_size_) + data_idx;
  } else {
    throw pcl::IOException("[pcl::PCDWriter::writeBinaryCompressed] Error during compression!");
    return nullptr;
  }

  data_size = sizeof(char) * compressed_final_size;
  char *data = (char *)malloc(data_size);

  // Copy the header
  memcpy(&data[0], oss.str().c_str(), data_idx);
  // Copy the compressed data
  memcpy(&data[data_idx], temp_buf, data_size_);

  free(only_valid_data);
  free(temp_buf);

  return data;
}

#endif /* BIND_IO_PCD_DATA_WRITER_IMPL_H_ */
