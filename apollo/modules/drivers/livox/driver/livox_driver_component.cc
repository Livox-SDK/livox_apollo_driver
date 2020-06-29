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

#include "modules/drivers/livox/driver/livox_driver_component.h"
#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace livox {

const uint64_t kSeconds = 1000000000; // ns

bool LivoxDriverComponent::Init() {
  AINFO << "Livox driver component init";
  Config livox_config;
  if (!GetProtoConfig(&livox_config)) {
    return false;
  }
  AINFO << "Livox config: " << livox_config.DebugString();

  if (!livox_config.has_lidars_conf()) {
    AERROR << "lidar_conf is empty!";
    return false;
  }

  if (livox_config.has_publish_frequency()) {
     publish_frequency_ = livox_config.publish_frequency();
  }

  publish_interval_ = (uint64_t)(kSeconds / publish_frequency_);

  for (const auto& lidar_conf : livox_config.lidars_conf().lidar_conf()) {
    auto channel_name = lidar_conf.convert_channel_name();
    uint8_t hub_port = (uint8_t)lidar_conf.hub_port();
    auto frame_id = lidar_conf.frame_id();

    auto writer = node_->CreateWriter<PointCloud>(channel_name);
    devices_[hub_port] = make_pair(frame_id, writer);

    point_cloud_time_last_[hub_port] = 0;

    std::shared_ptr<PointCloud> point_cloud = std::make_shared<PointCloud>();
    point_cloud->mutable_point()->Reserve(50000);
    point_cloud_pool_[hub_port] = point_cloud;
  }

  AINFO << "Point cloud comp convert init success";

  auto driver = LivoxDriverFactory::CreateDriver(livox_config);
  if (!driver) {
    return false;
  }

  // start the driver
  dvr_ = std::move(driver);
  if (!dvr_->DriverInit()) {
    return false;
  }
  dvr_->SetPointCloudCallback(std::bind(
      &LivoxDriverComponent::point_cloud_process, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  return true;
}

/** @brief Point cloud process. */
void LivoxDriverComponent::point_cloud_process(uint8_t hub_port,
                                               LivoxEthPacket* data,
                                               uint32_t data_num) {
  if (!data) {
    AWARN << "LivoxEthPacket is nullptr";
    return;
  }

  if (data->timestamp_type != kTimestampTypePpsGps) {
    AWARN << "point cloud timestamp type is not gps sync";
  }

  if (devices_.find(hub_port) == devices_.end()) {
    return;
  }

  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_[hub_port];

  if (!point_cloud_out) {
    AWARN << "point cloud out is nullptr";
    return;
  }

  //AINFO << "Lidar handle " << (int)hub_port;
  if (!dvr_->ConvertPacketsToPointcloud(data, data_num, point_cloud_out.get())) {
     return;
  }

  uint64_t point_cloud_time_now = dvr_->GetStoragePacketTimestamp(data);
  uint64_t point_cloud_time_last = point_cloud_time_last_[hub_port];
  if (point_cloud_time_last == 0) {
    point_cloud_time_last_[hub_port] = point_cloud_time_now;
    return;
  } else if (point_cloud_time_now - point_cloud_time_last < publish_interval_) {
    //AINFO << "now: " << point_cloud_time_now << "last: " << point_cloud_time_last;
    return;
  }

  point_cloud_time_last_[hub_port] = point_cloud_time_now;

  if (!point_cloud_out || point_cloud_out->point().empty()) {
    // AERROR << "point cloud has no point";
    return;
  } 

  auto frame_id = devices_[hub_port].first;
  auto writer = devices_[hub_port].second;

  point_cloud_out->mutable_header()->set_frame_id(frame_id);
  point_cloud_out->mutable_header()->set_timestamp_sec(
      cyber::Time().Now().ToSecond());

  writer->Write(point_cloud_out);
  point_cloud_out->Clear();
}

}  // namespace livox
}  // namespace drivers
}  // namespace apollo
