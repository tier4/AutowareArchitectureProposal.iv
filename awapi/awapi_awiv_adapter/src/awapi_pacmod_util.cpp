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

#include <awapi_awiv_adapter/awapi_pacmod_util.h>

namespace autoware_api
{
namespace pacmod_util
{
autoware_api_msgs::DoorStatus getDoorStatusMsg(const pacmod_msgs::SystemRptInt::ConstPtr & msg_ptr)
{
  autoware_api_msgs::DoorStatus door_status;

  if (!msg_ptr) {
    door_status.status = autoware_api_msgs::DoorStatus::NOT_APPLICABLE;
    return door_status;
  }

  door_status.status = autoware_api_msgs::DoorStatus::UNKNOWN;

  if (
    msg_ptr->command == pacmod_msgs::SystemRptInt::DOOR_CLOSE &&
    msg_ptr->output == pacmod_msgs::SystemRptInt::DOOR_OPEN) {
    // do not used (command & output are always the same value)
    door_status.status = autoware_api_msgs::DoorStatus::DOOR_CLOSING;
  } else if (
    msg_ptr->command == pacmod_msgs::SystemRptInt::DOOR_OPEN &&
    msg_ptr->output == pacmod_msgs::SystemRptInt::DOOR_CLOSE) {
    // do not used (command & output are always the same value)
    door_status.status = autoware_api_msgs::DoorStatus::DOOR_OPENING;
  } else if (msg_ptr->output == pacmod_msgs::SystemRptInt::DOOR_CLOSE) {
    door_status.status = autoware_api_msgs::DoorStatus::DOOR_CLOSED;
  } else if (msg_ptr->output == pacmod_msgs::SystemRptInt::DOOR_OPEN) {
    door_status.status = autoware_api_msgs::DoorStatus::DOOR_OPENED;
  }

  return door_status;
}

pacmod_msgs::SystemCmdInt createClearOverrideDoorCommand()
{
  pacmod_msgs::SystemCmdInt door_cmd;
  door_cmd.header.frame_id = "base_link";
  door_cmd.header.stamp = ros::Time::now();
  door_cmd.clear_override = true;
  return door_cmd;
}

pacmod_msgs::SystemCmdInt createDoorCommand(const std_msgs::Bool::ConstPtr & msg_ptr)
{
  pacmod_msgs::SystemCmdInt door_cmd;
  door_cmd.header.frame_id = "base_link";
  door_cmd.header.stamp = ros::Time::now();
  door_cmd.enable = true;

  if (!msg_ptr) return {};

  if (msg_ptr->data) {
    door_cmd.command = pacmod_msgs::SystemCmdInt::DOOR_OPEN;
  } else {
    door_cmd.command = pacmod_msgs::SystemCmdInt::DOOR_CLOSE;
  }
  return door_cmd;
}

}  // namespace pacmod_util

}  // namespace autoware_api
