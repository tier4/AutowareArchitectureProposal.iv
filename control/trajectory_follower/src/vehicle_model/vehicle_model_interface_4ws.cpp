// Copyright 2018-2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower/vehicle_model/vehicle_model_interface_4ws.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
VehicleModelInterface4ws::VehicleModelInterface4ws(
  int64_t dim_x, int64_t dim_u, int64_t dim_y,
  float64_t wheelbase)
: m_dim_x(dim_x), m_dim_u(dim_u), m_dim_y(dim_y), m_wheelbase(wheelbase)
{
}
int64_t VehicleModelInterface4ws::getDimX() {return m_dim_x;}
int64_t VehicleModelInterface4ws::getDimU() {return m_dim_u;}
int64_t VehicleModelInterface4ws::getDimY() {return m_dim_y;}
float64_t VehicleModelInterface4ws::getWheelbase() {return m_wheelbase;}
void VehicleModelInterface4ws::setVelocity(const float64_t velocity) {m_velocity = velocity;}
void VehicleModelInterface4ws::setCurvature(const float64_t curvature) {m_curvature = curvature;}
void VehicleModelInterface4ws::setPosture(const float64_t posture) {m_posture = posture;}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
