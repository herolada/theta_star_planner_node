// Copyright (c) 2026 Adam Herold
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

#ifndef THETA_STAR_PLANNER_NODE__COSTMAP_INFLATION_HPP_
#define THETA_STAR_PLANNER_NODE__COSTMAP_INFLATION_HPP_

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace theta_star_planner_node
{

/**
 * @brief Inflates LETHAL_OBSTACLE cells in place, following nav2_costmap_2d::InflationLayer's
 *        cost model (without needing a layered costmap):
 *          - distance <= inscribed_radius           -> INSCRIBED_INFLATED_OBSTACLE (253)
 *          - inscribed_radius < distance <= radius  -> 252 * exp(-cost_scaling_factor *
 *                                                      (distance - inscribed_radius))
 *        where radius = max(inflation_radius, inscribed_radius). Existing costs are only ever
 *        raised. NO_INFORMATION cells are only overwritten inside the inscribed radius
 *        (InflationLayer's inflate_unknown=false behaviour).
 *        Theta* treats cells with cost > 252 as untraversable, so the inscribed radius acts as
 *        a hard robot-radius clearance and the decaying band as a soft one.
 * @param inscribed_radius metres, typically the robot radius
 * @param inflation_radius metres; does nothing if both radii are <= 0
 */
void inflateCostmap(
  nav2_costmap_2d::Costmap2D & costmap,
  double inscribed_radius,
  double inflation_radius,
  double cost_scaling_factor);

}  // namespace theta_star_planner_node

#endif  // THETA_STAR_PLANNER_NODE__COSTMAP_INFLATION_HPP_
