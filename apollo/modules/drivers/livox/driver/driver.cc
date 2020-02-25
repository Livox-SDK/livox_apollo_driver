/******************************************************************************
 * Copyright 2020 Livox. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "livox_def.h"
#include "livox_sdk.h"

#include "cyber/cyber.h"
#include "modules/drivers/livox/driver/driver.h"

#include "modules/drivers/livox/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace livox {

using apollo::drivers::PointXYZIT;

/** @brief Convert LivoxEthPacket to PointCloud. */
void LivoxDriver::ConvertPacketsToPointcloud(LivoxEthPacket* data,
                                             uint32_t data_num,
                                             PointCloud* pc) {
  if (!pc || !data) {
    return;
  }

  switch (data->data_type) {
    case kCartesian:
      CartesianPointProcess(data, data_num, pc);
      break;
    case kExtendCartesian:
      ExtendCartesianPointProcess(data, data_num, pc);
      break;
    case kDualExtendCartesian:
      DualExtendCartesianPointProcess(data, data_num, pc);
      break;
    case kImu:
      // AINFO << "data_type is IMU";
      break;
    default:
      AERROR << "data_type is wrong";
      break;
  }
}

void LivoxDriver::CartesianPointProcess(LivoxEthPacket* data, uint32_t data_num,
                                        PointCloud* pc) {
  if (!pc || !data) {
    return;
  }

  const LivoxRawPoint* raw = (const LivoxRawPoint*)data->data;

  // add new point cloud
  for (uint32_t i = 0; i < data_num; i++) {
    PointXYZIT* point_new = pc->add_point();
    point_new->set_x(static_cast<float>(raw[i].x / 1000.0));
    point_new->set_y(static_cast<float>(raw[i].y / 1000.0));
    point_new->set_z(static_cast<float>(raw[i].z / 1000.0));
    point_new->set_intensity(raw[i].reflectivity);
    point_new->set_timestamp(*(uint64_t*)data->timestamp);
  }
  AINFO << "point x: " << raw[data_num - 1].x / 1000.0
        << " y: " << raw[data_num - 1].y / 1000.0
        << " z: " << raw[data_num - 1].z / 1000.0
        << " intesnsity: " << (int)raw[data_num - 1].reflectivity;
}

void LivoxDriver::ExtendCartesianPointProcess(LivoxEthPacket* data,
                                              uint32_t data_num,
                                              PointCloud* pc) {
  if (!pc || !data) {
    return;
  }

  const LivoxExtendRawPoint* raw = (const LivoxExtendRawPoint*)data->data;

  // add new point cloud
  for (uint32_t i = 0; i < data_num; i++) {
    PointXYZIT* point_new = pc->add_point();
    point_new->set_x(static_cast<float>(raw[i].x / 1000.0));
    point_new->set_y(static_cast<float>(raw[i].y / 1000.0));
    point_new->set_z(static_cast<float>(raw[i].z / 1000.0));
    point_new->set_intensity(raw[i].reflectivity);
    point_new->set_timestamp(*(uint64_t*)data->timestamp);
  }
  AINFO << "point x: " << raw[data_num - 1].x / 1000.0
        << " y: " << raw[data_num - 1].y / 1000.0
        << " z: " << raw[data_num - 1].z / 1000.0
        << " intesnsity: " << (int)raw[data_num - 1].reflectivity;
}

void LivoxDriver::DualExtendCartesianPointProcess(LivoxEthPacket* data,
                                                  uint32_t data_num,
                                                  PointCloud* pc) {
  if (!pc || !data) {
    return;
  }

  const LivoxDualExtendRawPoint* raw =
      (const LivoxDualExtendRawPoint*)data->data;

  // add new point cloud
  for (uint32_t i = 0; i < data_num; i++) {
    PointXYZIT* point_new = pc->add_point();
    point_new->set_x(static_cast<float>(raw[i].x1 / 1000.0));
    point_new->set_y(static_cast<float>(raw[i].y1 / 1000.0));
    point_new->set_z(static_cast<float>(raw[i].z1 / 1000.0));
    point_new->set_intensity(raw[i].reflectivity1);
    point_new->set_timestamp(*(uint64_t*)data->timestamp);

    point_new = pc->add_point();
    point_new->set_x(static_cast<float>(raw[i].x2 / 1000.0));
    point_new->set_y(static_cast<float>(raw[i].y2 / 1000.0));
    point_new->set_z(static_cast<float>(raw[i].z2 / 1000.0));
    point_new->set_intensity(raw[i].reflectivity2);
    point_new->set_timestamp(*(uint64_t*)data->timestamp);
  }
  AINFO << "point x: " << raw[data_num - 1].x1 / 1000.0
        << " y: " << raw[data_num - 1].y1 / 1000.0
        << " z: " << raw[data_num - 1].z1 / 1000.0
        << " intesnsity: " << (int)raw[data_num - 1].reflectivity1;
}

}  // namespace livox
}  // namespace drivers
}  // namespace apollo