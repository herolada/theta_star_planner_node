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

#include "theta_star_planner_node/costmap_inflation.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace theta_star_planner_node
{

namespace
{
struct KernelCell
{
  int dx, dy;
  unsigned char cost;
};
}  // namespace

void inflateCostmap(
  nav2_costmap_2d::Costmap2D & costmap,
  double inscribed_radius,
  double inflation_radius,
  double cost_scaling_factor)
{
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  using nav2_costmap_2d::NO_INFORMATION;

  inscribed_radius = std::max(0.0, inscribed_radius);
  const double radius = std::max(inflation_radius, inscribed_radius);
  if (radius <= 0.0) {
    return;
  }

  const double res = costmap.getResolution();
  const int r_cells = static_cast<int>(std::ceil(radius / res));

  // Precompute the stamp: every cell offset within `radius`, with its inflated cost.
  std::vector<KernelCell> kernel;
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const double d = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * res;
      if (d > radius) {
        continue;
      }
      unsigned char c;
      if (d <= inscribed_radius) {
        c = INSCRIBED_INFLATED_OBSTACLE;
      } else {
        const double factor = std::exp(-cost_scaling_factor * (d - inscribed_radius));
        c = static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
      }
      if (c > nav2_costmap_2d::FREE_SPACE) {
        kernel.push_back({dx, dy, c});
      }
    }
  }
  if (kernel.empty()) {
    return;
  }

  const int sx = static_cast<int>(costmap.getSizeInCellsX());
  const int sy = static_cast<int>(costmap.getSizeInCellsY());
  unsigned char * data = costmap.getCharMap();
  // Read lethal cells from an untouched copy so stamping order doesn't matter.
  const std::vector<unsigned char> base(data, data + static_cast<size_t>(sx) * sy);

  auto lethal = [&](int x, int y) {
      return base[static_cast<size_t>(y) * sx + x] == LETHAL_OBSTACLE;
    };

  for (int y = 0; y < sy; ++y) {
    for (int x = 0; x < sx; ++x) {
      if (!lethal(x, y)) {
        continue;
      }
      // Interior lethal cells (all 4-neighbours lethal) can't raise anything their border
      // neighbours won't already raise more strongly; skip them.
      if (x > 0 && x + 1 < sx && y > 0 && y + 1 < sy &&
        lethal(x - 1, y) && lethal(x + 1, y) && lethal(x, y - 1) && lethal(x, y + 1))
      {
        continue;
      }
      for (const auto & k : kernel) {
        const int nx = x + k.dx;
        const int ny = y + k.dy;
        if (nx < 0 || ny < 0 || nx >= sx || ny >= sy) {
          continue;
        }
        unsigned char & o = data[static_cast<size_t>(ny) * sx + nx];
        if (o == LETHAL_OBSTACLE) {
          continue;
        }
        if (o == NO_INFORMATION) {
          if (k.cost == INSCRIBED_INFLATED_OBSTACLE) {
            o = k.cost;
          }
          continue;
        }
        o = std::max(o, k.cost);
      }
    }
  }
}

}  // namespace theta_star_planner_node
