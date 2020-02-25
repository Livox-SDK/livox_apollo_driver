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

#pragma once

#include "livox_def.h"
#include "livox_sdk.h"

#include "modules/drivers/livox/driver/driver.h"
#include "modules/drivers/livox/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

#include <functional>

namespace apollo {
namespace drivers {
namespace livox {

class LivoxHubDriver : public LivoxDriver {
 public:
  explicit LivoxHubDriver(const Config &config) : LivoxDriver(config) {
    hub_handle_ = kMaxLidarCount - 1;
  };
  ~LivoxHubDriver() { DriverUninit(); };

  bool DriverInit() override;
  void DriverUninit() override;
  void HubStartSample();
  void PointCloudProcessCallback(uint8_t hub_handle, LivoxEthPacket *data,
                                 uint32_t data_num);

 private:
  uint8_t hub_handle_;
  std::vector<ConnectedLidarInfo> devices_;
  std::map<uint8_t, std::string> lidar_conf_;

  bool IsMid40(std::string sn);
  void HubQueryLidarInfo(std::function<void(bool)> callback);

  void HubConfig(std::function<void(bool)> callback);
  void HubConfigPointCloudReturnMode(std::function<void(bool)> callback);
  void HubConfigFan(std::function<void(bool)> callback);
  void HubConfigCoordinate(std::function<void(bool)> callback);
  void HubSampling();

  static void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type);
  static void HubDataCb(uint8_t hub_handle, LivoxEthPacket *data,
                        uint32_t data_num, void *client_data);
};

}  // namespace livox
}  // namespace drivers
}  // namespace apollo