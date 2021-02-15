/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AWAPI_PACMOD_UTIL_H
#define AWAPI_PACMOD_UTIL_H

#include <autoware_api_msgs/DoorStatus.h>
#include <pacmod_msgs/SystemCmdInt.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <std_msgs/Bool.h>

namespace autoware_api
{
namespace pacmod_util
{
autoware_api_msgs::DoorStatus getDoorStatusMsg(const pacmod_msgs::SystemRptInt::ConstPtr & msg_ptr);
pacmod_msgs::SystemCmdInt createClearOverrideDoorCommand();
pacmod_msgs::SystemCmdInt createDoorCommand(const std_msgs::Bool::ConstPtr & msg_ptr);
}  // namespace pacmod_util

}  // namespace autoware_api

#endif  // AWAPI_AUTOWARE_UTIL_H