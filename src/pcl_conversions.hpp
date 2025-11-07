#pragma once
#include <cstdint>
#include <vector>
#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>               
#include <pcl_conversions/pcl_conversions.h>

namespace pcl_df {

// Versión ROS 2 del helper
template<typename T>
inline void fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud,
                       pcl::PointCloud<T> &pcl_cloud)
{
  // Copia metadatos
  pcl_conversions::toPCL(cloud.header, pcl_cloud.header);
  pcl_cloud.width    = cloud.width;
  pcl_cloud.height   = cloud.height;
  pcl_cloud.is_dense = (cloud.is_dense == 1);

  // Mapeo de campos msg -> struct T
  pcl::MsgFieldMap field_map;
  std::vector<pcl::PCLPointField> msg_fields;
  pcl_conversions::toPCL(cloud.fields, msg_fields);
  pcl::createMapping<T>(msg_fields, field_map);

  // Reserva/resize puntos
  const std::uint32_t num_points = cloud.width * cloud.height;
  pcl_cloud.points.resize(num_points);
  std::uint8_t *cloud_data = reinterpret_cast<std::uint8_t*>(&pcl_cloud.points[0]);

  // Ruta rápida: layout idéntico (un solo memcpy grande)
  if (field_map.size() == 1 &&
      field_map[0].serialized_offset == 0 &&
      field_map[0].struct_offset == 0 &&
      field_map[0].size == cloud.point_step &&
      field_map[0].size == sizeof(T))
  {
    const std::uint32_t cloud_row_step = static_cast<std::uint32_t>(sizeof(T) * pcl_cloud.width);
    const std::uint8_t *msg_data = cloud.data.data();

    if (cloud.row_step == cloud_row_step) {
      std::memcpy(cloud_data, msg_data, cloud.data.size());
    } else {
      for (std::uint32_t r = 0; r < cloud.height; ++r,
           cloud_data += cloud_row_step, msg_data += cloud.row_step)
      {
        std::memcpy(cloud_data, msg_data, cloud_row_step);
      }
    }
    return;
  }

  // Ruta general: copiar por campos mapeados
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::uint8_t *row_data = &cloud.data[row * cloud.row_step];
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::uint8_t *msg_data = row_data + col * cloud.point_step;
      for (const pcl::detail::FieldMapping &m : field_map) {
        std::memcpy(cloud_data + m.struct_offset, msg_data + m.serialized_offset, m.size);
      }
      cloud_data += sizeof(T);
    }
  }
}

} // namespace pcl_df
