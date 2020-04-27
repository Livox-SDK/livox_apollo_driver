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

#include "modules/drivers/livox/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

#include <functional>

namespace apollo {
namespace drivers {
namespace livox {

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;

class LivoxDriver {
 public:
  explicit LivoxDriver(const Config &config) : config_(config){};
  virtual ~LivoxDriver() = default;
  virtual bool DriverInit() = 0;
  virtual void DriverUninit() = 0;

  void ConvertPacketsToPointcloud(LivoxEthPacket *data, uint32_t data_num,
                                  PointCloud *pc);

  using PointCloudCallback = std::function<void(
      uint8_t hub_port, LivoxEthPacket *data, uint32_t data_num)>;
  void SetPointCloudCallback(const PointCloudCallback &data_cb) {
    data_cb_ = data_cb;
  };

 protected:
  Config config_;
  PointCloudCallback data_cb_;

 private:
  void DualExtendCartesianPointProcess(LivoxEthPacket *data, uint32_t data_num,
                                       PointCloud *pc);
  void ExtendCartesianPointProcess(LivoxEthPacket *data, uint32_t data_num,
                                   PointCloud *pc);
  void CartesianPointProcess(LivoxEthPacket *data, uint32_t data_num,
                             PointCloud *pc);
};

}  // namespace livox
}  // namespace drivers
}  // namespace apollo