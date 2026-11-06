// Copyright 2020 Anshumaan Singh
// Copyright (c) 2026 Adam Herold
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Adapted from nav2_theta_star_planner/src/theta_star.cpp (ros-navigation/navigation2,
// kilted). See theta_star.hpp for the list of changes.

#include "theta_star_planner_node/theta_star.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace theta_star_planner_node
{

ThetaStar::ThetaStar(const ThetaStarParams & params)
{
  setParams(params);
}

void ThetaStar::setParams(const ThetaStarParams & params)
{
  params_ = params;
  if (params_.how_many_corners != 8 && params_.how_many_corners != 4) {
    params_.how_many_corners = 8;
  }
  // Admissible/consistent heuristic, as in upstream ThetaStarPlanner::configure.
  w_heuristic_cost_ = std::min(params_.w_euc_cost, 1.0);
}

void ThetaStar::setCostmap(const nav2_costmap_2d::Costmap2D * costmap)
{
  costmap_ = costmap;
  charmap_ = costmap_->getCharMap();
  size_x_ = static_cast<int>(costmap_->getSizeInCellsX());
  size_y_ = static_cast<int>(costmap_->getSizeInCellsY());
}

void ThetaStar::setStartAndGoal(const CoordsM & start, const CoordsM & goal)
{
  src_ = start;
  dst_ = goal;
}

SearchStatus ThetaStar::generatePath(
  std::vector<CoordsW> & raw_path, const std::function<bool()> & cancel_checker)
{
  raw_path.clear();
  resetContainers();
  nodes_opened_ = 0;

  const auto t0 = std::chrono::steady_clock::now();
  const int check_interval = std::max(1, params_.terminal_checking_interval);

  TreeNode * src = addNode(src_.x, src_.y);
  src->g = getTraversalCost(src_.x, src_.y);
  src->h = getHCost(src_.x, src_.y);
  src->f = src->g + src->h;
  src->parent_id = src;
  src->is_in_queue = true;
  queue_.push(src);

  const TreeNode * goal_node = nullptr;
  const TreeNode * best_node = nullptr;   // closest expanded node within tolerance
  double best_dist = std::numeric_limits<double>::max();
  SearchStatus fail_status = SearchStatus::NO_PATH;

  while (!queue_.empty()) {
    TreeNode * curr_data = queue_.top();
    queue_.pop();
    nodes_opened_++;

    if (nodes_opened_ % check_interval == 0) {
      if (cancel_checker && cancel_checker()) {
        queue_ = decltype(queue_)();
        return SearchStatus::CANCELED;
      }
      if (params_.max_planning_time > 0.0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >
        params_.max_planning_time)
      {
        fail_status = SearchStatus::TIMEOUT;
        break;
      }
    }
    if (params_.max_iterations > 0 && nodes_opened_ > params_.max_iterations) {
      fail_status = SearchStatus::TIMEOUT;
      break;
    }

    resetParent(curr_data);

    if (isGoal(*curr_data)) {
      goal_node = curr_data;
      break;
    }

    if (params_.tolerance_cells > 0.0) {
      const double d = std::hypot(curr_data->x - dst_.x, curr_data->y - dst_.y);
      if (d <= params_.tolerance_cells && d < best_dist) {
        best_dist = d;
        best_node = curr_data;
      }
    }

    setNeighbors(curr_data);
  }
  queue_ = decltype(queue_)();

  if (goal_node) {
    backtrace(raw_path, goal_node);
    return SearchStatus::GOAL_REACHED;
  }
  if (best_node) {
    backtrace(raw_path, best_node);
    return SearchStatus::WITHIN_TOLERANCE;
  }
  return fail_status;
}

TreeNode * ThetaStar::addNode(int cx, int cy)
{
  // nodes_data_ is reserved to the cell count in resetContainers() and every cell gets at
  // most one node, so this push_back never reallocates (and never invalidates pointers).
  nodes_data_.push_back(TreeNode{cx, cy});
  TreeNode * n = &nodes_data_.back();
  node_position_[size_x_ * cy + cx] = n;
  return n;
}

void ThetaStar::resetParent(TreeNode * curr_data)
{
  double g_cost, los_cost = 0;
  curr_data->is_in_queue = false;
  const TreeNode * curr_par = curr_data->parent_id;
  const TreeNode * maybe_par = curr_par->parent_id;

  if (losCheck(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y, los_cost)) {
    g_cost = maybe_par->g +
      getEuclideanCost(curr_data->x, curr_data->y, maybe_par->x, maybe_par->y) + los_cost;

    if (g_cost < curr_data->g) {
      curr_data->parent_id = maybe_par;
      curr_data->g = g_cost;
      curr_data->f = g_cost + curr_data->h;
    }
  }
}

void ThetaStar::setNeighbors(const TreeNode * curr_data)
{
  for (int i = 0; i < params_.how_many_corners; i++) {
    const int mx = curr_data->x + moves[i].x;
    const int my = curr_data->y + moves[i].y;

    if (!withinLimits(mx, my) || !isSafe(mx, my)) {
      continue;
    }
    // No corner cutting: a diagonal move passes through the corner shared with both
    // orthogonal neighbours, so both must be safe (upstream only checked the target cell).
    if (moves[i].x != 0 && moves[i].y != 0 &&
      (!isSafe(curr_data->x + moves[i].x, curr_data->y) ||
      !isSafe(curr_data->x, curr_data->y + moves[i].y)))
    {
      continue;
    }

    const double g_cost = curr_data->g + getEuclideanCost(curr_data->x, curr_data->y, mx, my) +
      getTraversalCost(mx, my);

    TreeNode * exp_node = getIndex(mx, my);
    if (exp_node == nullptr) {
      exp_node = addNode(mx, my);
    }

    const double h_cost = getHCost(mx, my);
    const double cal_cost = g_cost + h_cost;
    if (exp_node->f > cal_cost) {
      exp_node->g = g_cost;
      exp_node->h = h_cost;
      exp_node->f = cal_cost;
      exp_node->parent_id = curr_data;
      if (!exp_node->is_in_queue) {
        exp_node->is_in_queue = true;
        queue_.push(exp_node);
      }
    }
  }
}

void ThetaStar::backtrace(std::vector<CoordsW> & raw_points, const TreeNode * curr_n) const
{
  std::vector<CoordsW> path_rev;
  CoordsW world{};
  while (true) {
    costmap_->mapToWorld(
      static_cast<unsigned int>(curr_n->x), static_cast<unsigned int>(curr_n->y),
      world.x, world.y);
    path_rev.push_back(world);
    if (curr_n->parent_id == curr_n) {
      break;
    }
    curr_n = curr_n->parent_id;
  }
  raw_points.assign(path_rev.rbegin(), path_rev.rend());
}

bool ThetaStar::losCheck(int x0, int y0, int x1, int y1, double & sl_cost) const
{
  // Walks exactly the cells crossed by the segment between the *centers* of cells
  // (x0, y0) and (x1, y1) (which is where backtrace() puts the path vertices), summing the
  // traversal cost of every entered cell (start excluded, end included, as neighbour
  // expansion does). Upstream checked the line between cell *corners* instead, so the
  // center-to-center path it produced could cut through lethal cells by up to half a cell.
  // When the segment passes exactly through a cell corner, both cells beside it must be
  // safe too (they are touched, though not entered).
  sl_cost = 0;

  const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  const int sx = x1 > x0 ? 1 : -1;
  const int sy = y1 > y0 ? 1 : -1;
  int x = x0, y = y0;
  int ix = 0, iy = 0;   // vertical / horizontal cell borders crossed so far

  while (ix < dx || iy < dy) {
    // The next vertical border is crossed at t = (2 ix + 1) / (2 dx), the next horizontal
    // one at t = (2 iy + 1) / (2 dy); compare them in integers.
    const int64_t tx = static_cast<int64_t>(2 * ix + 1) * dy;
    const int64_t ty = static_cast<int64_t>(2 * iy + 1) * dx;
    if (dy == 0 || (dx != 0 && tx < ty)) {
      x += sx;
      ix++;
    } else if (dx == 0 || ty < tx) {
      y += sy;
      iy++;
    } else {
      if (!withinLimits(x + sx, y) || !isSafe(x + sx, y) ||
        !withinLimits(x, y + sy) || !isSafe(x, y + sy))
      {
        return false;
      }
      x += sx;
      y += sy;
      ix++;
      iy++;
    }
    if (!isSafe(x, y, sl_cost)) {
      return false;
    }
  }
  return true;
}

void ThetaStar::resetContainers()
{
  const size_t num_cells = static_cast<size_t>(size_x_) * static_cast<size_t>(size_y_);
  node_position_.assign(num_cells, nullptr);
  nodes_data_.clear();
  nodes_data_.reserve(num_cells);
  queue_ = decltype(queue_)();
}

std::vector<CoordsW> ThetaStar::linearInterpolation(
  const std::vector<CoordsW> & raw_path, double dist_bw_points)
{
  std::vector<CoordsW> out;
  if (raw_path.empty()) {
    return out;
  }
  for (size_t j = 0; j + 1 < raw_path.size(); j++) {
    const CoordsW & pt1 = raw_path[j];
    const CoordsW & pt2 = raw_path[j + 1];
    out.push_back(pt1);

    const double distance = std::hypot(pt2.x - pt1.x, pt2.y - pt1.y);
    if (dist_bw_points <= 0.0 || distance <= 0.0) {
      continue;
    }
    const int loops = static_cast<int>(distance / dist_bw_points);
    const double sin_alpha = (pt2.y - pt1.y) / distance;
    const double cos_alpha = (pt2.x - pt1.x) / distance;
    for (int k = 1; k < loops; k++) {
      out.push_back(
        {pt1.x + k * dist_bw_points * cos_alpha, pt1.y + k * dist_bw_points * sin_alpha});
    }
  }
  out.push_back(raw_path.back());
  return out;
}

}  // namespace theta_star_planner_node
