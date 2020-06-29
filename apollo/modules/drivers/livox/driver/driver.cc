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
bool LivoxDriver::ConvertPacketsToPointcloud(LivoxEthPacket* data,
                                             uint32_t data_num,
                                             PointCloud* pc) {
  if (!pc || !data) {
    return false;
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
      return false;
      break;
    default:
      AERROR << "data_type is wrong";
      return false;
  }
  return true;
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
  // AINFO << "point x: " << raw[data_num - 1].x / 1000.0
  //       << " y: " << raw[data_num - 1].y / 1000.0
  //       << " z: " << raw[data_num - 1].z / 1000.0
  //       << " intesnsity: " << (int)raw[data_num - 1].reflectivity;
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
  // AINFO << "point x: " << raw[data_num - 1].x / 1000.0
  //       << " y: " << raw[data_num - 1].y / 1000.0
  //       << " z: " << raw[data_num - 1].z / 1000.0
  //       << " intesnsity: " << (int)raw[data_num - 1].reflectivity;
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
  // AINFO << "point x: " << raw[data_num - 1].x1 / 1000.0
  //       << " y: " << raw[data_num - 1].y1 / 1000.0
  //       << " z: " << raw[data_num - 1].z1 / 1000.0
  //       << " intesnsity: " << (int)raw[data_num - 1].reflectivity1;
}

uint64_t LivoxDriver::GetStoragePacketTimestamp(LivoxEthPacket *raw_packet)
{
  LdsStamp timestamp;
  memcpy(timestamp.stamp_bytes, raw_packet->timestamp, sizeof(timestamp));

  if (raw_packet->timestamp_type == kTimestampTypePps) {
      return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypeNoSync) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePtp) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePpsGps) {
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    time_utc.tm_year = raw_packet->timestamp[0] + 100; // map 2000 to 1990
    time_utc.tm_mon  = raw_packet->timestamp[1] - 1;   // map 1~12 to 0~11
    time_utc.tm_mday = raw_packet->timestamp[2];
    time_utc.tm_hour = raw_packet->timestamp[3];
    time_utc.tm_min = 0;
    time_utc.tm_sec = 0;

    uint64_t time_epoch = timegm(&time_utc); // no timezone
    time_epoch = time_epoch * 1000000 + timestamp.stamp_word.high; // to us
    time_epoch = time_epoch * 1000;                                // to ns

    return time_epoch;
  } else {
    printf("Timestamp type[%d] invalid.\n", raw_packet->timestamp_type);
    return 0;
  }
}

}  // namespace livox
}  // namespace drivers
}  // namespace apollo