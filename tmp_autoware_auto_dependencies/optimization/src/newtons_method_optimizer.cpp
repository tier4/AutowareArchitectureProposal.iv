// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "optimization/newtons_method_optimizer.hpp"
#include "optimization/line_search/fixed_line_search.hpp"
#include "optimization/line_search/more_thuente_line_search.hpp"

namespace autoware
{
namespace common
{
namespace optimization
{
// Instantiation of the optimizer with concrete types.
template class NewtonsMethodOptimizer<FixedLineSearch>;
template class NewtonsMethodOptimizer<MoreThuenteLineSearch>;
}  // namespace optimization
}  // namespace common
}  // namespace autoware
