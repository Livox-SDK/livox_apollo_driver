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

  for (const auto& lidar_conf : livox_config.lidars_conf().lidar_conf()) {
    auto channel_name = lidar_conf.convert_channel_name();
    uint8_t hub_port = (uint8_t)lidar_conf.hub_port();
    auto frame_id = lidar_conf.frame_id();

    auto writer = node_->CreateWriter<PointCloud>(channel_name);
    devices_[hub_port] = make_pair(frame_id, writer);
  }

  point_cloud_pool_.reset(new CCObjectPool<PointCloud>(pool_size_));
  point_cloud_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; i++) {
    auto point_cloud = point_cloud_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject, i: " << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(10000);
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

  std::shared_ptr<PointCloud> point_cloud_out = point_cloud_pool_->GetObject();
  if (!point_cloud_out) {
    AWARN << "point cloud pool return nullptr, will be create new.";
    point_cloud_out = std::make_shared<PointCloud>();
    point_cloud_out->mutable_point()->Reserve(10000);
  }
  if (!point_cloud_out) {
    AWARN << "point cloud out is nullptr";
    return;
  }

  point_cloud_out->Clear();

  if (devices_.find(hub_port) == devices_.end()) {
    return;
  }

  auto frame_id = devices_[hub_port].first;
  auto writer = devices_[hub_port].second;

  point_cloud_out->mutable_header()->set_frame_id(frame_id);
  point_cloud_out->mutable_header()->set_timestamp_sec(
      cyber::Time().Now().ToSecond());

  dvr_->ConvertPacketsToPointcloud(data, data_num, point_cloud_out.get());

  if (!point_cloud_out || point_cloud_out->point().empty()) {
    // AERROR << "point cloud has no point";
    return;
  }

  writer->Write(point_cloud_out);
}

}  // namespace livox
}  // namespace drivers
}  // namespace apollo
