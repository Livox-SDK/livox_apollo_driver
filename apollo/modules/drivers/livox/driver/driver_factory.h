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

#include "modules/drivers/livox//driver/driver.h"
#include "modules/drivers/livox//driver/hub_driver.h"

namespace apollo {
namespace drivers {
namespace livox {

class LivoxDriverFactory {
 public:
  static std::unique_ptr<LivoxDriver> CreateDriver(const Config &config) {
    if (config.has_use_hub() && config.use_hub()) {
      return std::unique_ptr<LivoxDriver>(new LivoxHubDriver(config));
    } else {
      return nullptr;
    }
  }
};

}  // namespace livox
}  // namespace drivers
}  // namespace apollo