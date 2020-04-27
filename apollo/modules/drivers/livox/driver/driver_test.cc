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

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/livox/driver/driver_factory.h"
#include "modules/drivers/livox/proto/config.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"

#include "gtest/gtest.h"

using apollo::cyber::base::CCObjectPool;
using apollo::drivers::PointCloud;

namespace apollo {
namespace drivers {
namespace livox {

TEST(LivoxDriverTest, General) {
  AINFO << "Livox driver component init";
  Config config;
  config.set_use_hub(true);
  LidarConfig *lidars_config = config.mutable_lidars_conf();
  LidarConfigInfo *lidar_conf_info = lidars_config->add_lidar_conf();
  lidar_conf_info->set_return_mode(FirstReturn);
  lidar_conf_info->set_hub_port(1);
  lidar_conf_info->set_fan_status(true);

  lidar_conf_info = lidars_config->add_lidar_conf();
  lidar_conf_info->set_return_mode(FirstReturn);
  lidar_conf_info->set_hub_port(2);
  lidar_conf_info->set_fan_status(true);

  auto driver = LivoxDriverFactory::CreateDriver(config);
  EXPECT_NE(driver, nullptr);

  EXPECT_TRUE(driver->DriverInit());
  driver->SetPointCloudCallback([&](uint8_t hub_port, LivoxEthPacket *data,
                                    uint32_t data_num) {
    EXPECT_NE(data, nullptr);
    std::shared_ptr<PointCloud> point_cloud = std::make_shared<PointCloud>();
    point_cloud->mutable_point()->Reserve(10000);
    driver->ConvertPacketsToPointcloud(data, data_num, point_cloud.get());
    if (data->data_type == kDualExtendCartesian ||
        data->data_type == kDualExtendSpherical) {
      EXPECT_EQ(point_cloud->point_size(), data_num * 2);
    } else if (data->data_type == kExtendCartesian ||
               data->data_type == kExtendSpherical ||
               data->data_type == kCartesian || data->data_type == kSpherical) {
      EXPECT_EQ(point_cloud->point_size(), data_num);
    }
    point_cloud->Clear();
  });
  std::this_thread::sleep_for(std::chrono::seconds(20));
}

}  // namespace livox
}  // namespace drivers
}  // namespace apollo

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}