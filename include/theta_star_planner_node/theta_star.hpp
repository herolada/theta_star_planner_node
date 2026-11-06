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
// Adapted from nav2_theta_star_planner/include/nav2_theta_star_planner/theta_star.hpp
// (ros-navigation/navigation2, kilted). Changes vs. upstream:
//  - ROS-message-free API: start/goal are given in map cells, the search returns a
//    SearchStatus instead of throwing nav2_core exceptions (no nav2_core dependency).
//  - The costmap is only read, never written: upstream's clearStart() (which set the
//    start cell to FREE_SPACE in the shared costmap) is replaced by treating the start
//    cell as free during the search.
//  - Search containers are reset for the exact current map size on every search, fixing
//    stale node pointers upstream could keep when the costmap size changes between plans
//    (here it changes on nearly every traversability message).
//  - losCheck() walks the cells crossed by the segment between cell *centers* (where the
//    path vertices are placed) instead of between cell corners, and diagonal neighbour
//    moves may not cut between two unsafe orthogonal cells. Upstream's corner-based check
//    let center-to-center paths cut through lethal cells by up to half a cell; with these
//    changes every returned path segment only crosses safe cells.
//  - Fixed the goal being reported as unreachable when it was the last node in the queue.
//  - The goal node also gets its lazy line-of-sight parent check (upstream skipped it,
//    leaving the last path segment non-taut).
//  - Added goal tolerance (closest expanded node within `tolerance` cells of the goal is
//    returned when the goal itself can't be reached), max_iterations and max_planning_time
//    limits, mirroring nav2_smac_planner's AStarAlgorithm semantics.
//  - linearInterpolation() keeps the final path vertex (upstream dropped it).
// The search itself (cost function, neighbour expansion, lazy parent reset) is unchanged.

#ifndef THETA_STAR_PLANNER_NODE__THETA_STAR_HPP_
#define THETA_STAR_PLANNER_NODE__THETA_STAR_HPP_

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace theta_star_planner_node
{

struct CoordsM
{
  int x, y;
};

struct CoordsW
{
  double x, y;
};

struct TreeNode
{
  int x, y;
  double g = std::numeric_limits<double>::max();
  double h = std::numeric_limits<double>::max();
  const TreeNode * parent_id = nullptr;
  bool is_in_queue = false;
  double f = std::numeric_limits<double>::max();
};

struct ThetaStarParams
{
  /// weight on the costmap traversal cost
  double w_traversal_cost = 2.0;
  /// weight on the euclidean distance cost (used for calculations of g_cost)
  double w_euc_cost = 1.0;
  /// number of adjacent nodes to be searched on (4 or 8)
  int how_many_corners = 8;
  /// whether the planner can plan through unknown space
  bool allow_unknown = true;
  /// the interval (in expanded nodes) at which cancellation and the time limit are checked
  int terminal_checking_interval = 5000;
  /// maximum number of expanded nodes, <= 0 means unlimited
  int max_iterations = 0;
  /// maximum search time in seconds, <= 0 means unlimited
  double max_planning_time = 0.0;
  /// if the goal can't be reached, accept the closest expanded node within this many cells
  double tolerance_cells = 0.0;
};

enum class SearchStatus
{
  GOAL_REACHED,
  WITHIN_TOLERANCE,   ///< goal not reached, path ends at the closest node within tolerance
  NO_PATH,            ///< open list exhausted, nothing within tolerance
  TIMEOUT,            ///< max_iterations or max_planning_time hit, nothing within tolerance
  CANCELED,
};

class ThetaStar
{
public:
  static constexpr unsigned char UNKNOWN_COST = 255;
  static constexpr unsigned char OCCUPIED_COST = 254;
  static constexpr unsigned char MAX_NON_OBSTACLE_COST = 252;

  explicit ThetaStar(const ThetaStarParams & params = ThetaStarParams());

  void setParams(const ThetaStarParams & params);
  const ThetaStarParams & getParams() const {return params_;}

  /**
   * @brief Sets the costmap to plan on. It is only read, and must outlive (and stay
   *        unmodified during) generatePath().
   */
  void setCostmap(const nav2_costmap_2d::Costmap2D * costmap);

  /**
   * @brief Sets the start and goal cells. Both must lie within the costmap.
   *        The start cell is treated as free space during the search.
   */
  void setStartAndGoal(const CoordsM & start, const CoordsM & goal);

  /**
   * @brief whether the cell's cost is <= MAX_NON_OBSTACLE_COST (or unknown and allow_unknown)
   */
  bool isSafe(int cx, int cy) const
  {
    const unsigned char c = cost(cx, cy);
    return (c == UNKNOWN_COST && params_.allow_unknown) || c <= MAX_NON_OBSTACLE_COST;
  }

  /**
   * @brief runs the search from start to goal
   * @param raw_path path vertices (world coordinates, start to goal) on success
   *        (GOAL_REACHED / WITHIN_TOLERANCE), empty otherwise
   * @param cancel_checker polled every terminal_checking_interval expansions
   */
  SearchStatus generatePath(
    std::vector<CoordsW> & raw_path, const std::function<bool()> & cancel_checker);

  /// number of nodes expanded by the last generatePath() call
  int nodesOpened() const {return nodes_opened_;}

  /**
   * @brief interpolates points between consecutive path vertices, `dist_bw_points` apart,
   *        keeping every vertex (including the last one)
   */
  static std::vector<CoordsW> linearInterpolation(
    const std::vector<CoordsW> & raw_path, double dist_bw_points);

protected:
  struct Comp
  {
    bool operator()(const TreeNode * p1, const TreeNode * p2) const
    {
      return p1->f > p2->f;
    }
  };

  /// raw costmap byte at (cx, cy), with the start cell reported as free space
  inline unsigned char cost(int cx, int cy) const
  {
    if (cx == src_.x && cy == src_.y) {
      return nav2_costmap_2d::FREE_SPACE;
    }
    return charmap_[size_x_ * cy + cx];
  }

  /// scales the costmap cost by shifting the origin to 26 and multiplying by 0.9
  inline double getCost(int cx, int cy) const
  {
    return 26 + 0.9 * cost(cx, cy);
  }

  /// w_traversal_cost * getCost^2 / MAX_NON_OBSTACLE_COST^2
  inline double getTraversalCost(int cx, int cy) const
  {
    const double curr_cost = getCost(cx, cy);
    return params_.w_traversal_cost * curr_cost * curr_cost / MAX_NON_OBSTACLE_COST /
           MAX_NON_OBSTACLE_COST;
  }

  /**
   * @brief isSafe() overload used by the LOS check, accumulating the traversal cost of
   *        (cx, cy) into `cost_acc`. Out-of-bounds cells are unsafe. Uses the same safety
   *        predicate as isSafe(cx, cy) (upstream additionally rejected cost-252 cells here);
   *        unknown cells cost as OCCUPIED_COST - 1, as upstream.
   */
  inline bool isSafe(int cx, int cy, double & cost_acc) const
  {
    if (!withinLimits(cx, cy) || !isSafe(cx, cy)) {
      return false;
    }
    const double curr_cost = cost(cx, cy) == UNKNOWN_COST ? OCCUPIED_COST - 1 : getCost(cx, cy);
    cost_acc += params_.w_traversal_cost * curr_cost * curr_cost / MAX_NON_OBSTACLE_COST /
      MAX_NON_OBSTACLE_COST;
    return true;
  }

  inline double getEuclideanCost(int ax, int ay, int bx, int by) const
  {
    return params_.w_euc_cost * std::hypot(ax - bx, ay - by);
  }

  inline double getHCost(int cx, int cy) const
  {
    return w_heuristic_cost_ * std::hypot(cx - dst_.x, cy - dst_.y);
  }

  inline bool withinLimits(int cx, int cy) const
  {
    return cx >= 0 && cx < size_x_ && cy >= 0 && cy < size_y_;
  }

  inline bool isGoal(const TreeNode & n) const
  {
    return n.x == dst_.x && n.y == dst_.y;
  }

  /// returns the node stored for (cx, cy), or nullptr if not yet visited
  inline TreeNode * getIndex(int cx, int cy)
  {
    return node_position_[size_x_ * cy + cx];
  }

  /// allocates a fresh node for (cx, cy) and registers it in node_position_
  TreeNode * addNode(int cx, int cy);

  /// line of sight check between the current node and its grandparent; reparents on success
  void resetParent(TreeNode * curr_data);

  /// expands the current node
  void setNeighbors(const TreeNode * curr_data);

  /// line of sight check between cell centers, accumulating the straight-line traversal cost
  bool losCheck(int x0, int y0, int x1, int y1, double & sl_cost) const;

  /// backtracks from `curr_n` to the start, writing start-to-curr_n world vertices
  void backtrace(std::vector<CoordsW> & raw_points, const TreeNode * curr_n) const;

  /// sizes and clears all search containers for the current costmap
  void resetContainers();

  ThetaStarParams params_;
  double w_heuristic_cost_ = 1.0;

  const nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  const unsigned char * charmap_ = nullptr;
  int size_x_ = 0, size_y_ = 0;
  CoordsM src_{}, dst_{};
  int nodes_opened_ = 0;

  /// node_position_[size_x_ * y + x] points at that cell's node in nodes_data_ (or nullptr)
  std::vector<TreeNode *> node_position_;
  /// storage for all visited nodes; reserved to the cell count so pointers stay valid
  std::vector<TreeNode> nodes_data_;
  /// open list
  std::priority_queue<TreeNode *, std::vector<TreeNode *>, Comp> queue_;

  const CoordsM moves[8] = {{0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
    {1, -1},
    {-1, 1},
    {1, 1},
    {-1, -1}};
};

}  // namespace theta_star_planner_node

#endif  // THETA_STAR_PLANNER_NODE__THETA_STAR_HPP_
