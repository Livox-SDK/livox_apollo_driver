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

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"

#include "modules/drivers/livox/driver/driver_factory.h"
#include "modules/drivers/livox/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace livox {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::PointCloud;

class LivoxDriverComponent : public Component<> {
 public:
  ~LivoxDriverComponent() {}
  bool Init() override;

 private:
  void point_cloud_process(uint8_t hub_port, LivoxEthPacket *data,
                           uint32_t data_num);
  std::shared_ptr<CCObjectPool<PointCloud>> point_cloud_pool_ = nullptr;
  int pool_size_ = 8;
  std::unique_ptr<LivoxDriver> dvr_;  ///< driver implementation class
  std::map<uint8_t,
           std::pair<std::string, std::shared_ptr<Writer<PointCloud>>>>
      devices_;
};

CYBER_REGISTER_COMPONENT(LivoxDriverComponent)

}  // namespace livox
}  // namespace drivers
}  // namespace apollo