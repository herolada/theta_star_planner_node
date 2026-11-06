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

// Standalone Theta* planner node.
//
// This node drives a vendored, ROS-free copy of nav2_theta_star_planner's ThetaStar search
// (see theta_star.hpp) directly, without the rest of the Nav2 stack (no Costmap2DROS, no
// lifecycle manager, no planner server). It builds its planning costmap from a
// traversability PointCloud2 and exposes planning through a ComputePathToPose action server
// (geometry_msgs/PoseStamped goal in, nav_msgs/Path out).
//
// Its ROS interface (topics, action, TF usage, costmap-construction parameters, error codes)
// is identical to smac_planner_node so the two are interchangeable in the navstack. The
// planner-specific logic mirrors nav2_theta_star_planner/src/theta_star_planner.cpp.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "theta_star_planner_node/theta_star.hpp"
#include "theta_star_planner_node/costmap_inflation.hpp"

namespace theta_star_planner_node
{

using namespace std::chrono_literals;  // NOLINT
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

/**
 * @class ThetaStarPlannerNode
 * @brief Standalone Theta* planner: traversability PointCloud2 -> costmap ->
 *        ThetaStar -> Path, served over a ComputePathToPose action.
 */
class ThetaStarPlannerNode : public rclcpp::Node
{
public:
  explicit ThetaStarPlannerNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("theta_star_planner_node", options)
  {
    declareAndGetParameters();
    _planner = std::make_unique<ThetaStar>(searchParams());

    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    _traversability_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      _traversability_topic, rclcpp::SensorDataQoS(),
      std::bind(&ThetaStarPlannerNode::traversabilityCallback, this, std::placeholders::_1));

    _plan_publisher = this->create_publisher<nav_msgs::msg::Path>("~/plan", 1);
    _raw_plan_publisher = this->create_publisher<nav_msgs::msg::Path>("~/plan_raw", 1);
    _costmap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "~/costmap", rclcpp::QoS(1).transient_local());
    _planning_start_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/planning_start", rclcpp::QoS(1).transient_local());
    _planning_goal_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/planning_goal", rclcpp::QoS(1).transient_local());

    _dyn_params_handler = this->add_on_set_parameters_callback(
      std::bind(&ThetaStarPlannerNode::dynamicParametersCallback, this, std::placeholders::_1));

    _action_server = rclcpp_action::create_server<ComputePathToPose>(
      this, _action_name,
      std::bind(
        &ThetaStarPlannerNode::handleGoal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&ThetaStarPlannerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&ThetaStarPlannerNode::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "theta_star_planner_node ready. Listening for traversability on '%s', "
      "serving ComputePathToPose on '%s'.",
      _traversability_topic.c_str(), _action_name.c_str());
  }

private:
  // ----------------------------------------------------------------------- //
  // Setup
  // ----------------------------------------------------------------------- //
  void declareAndGetParameters()
  {
    // Topics / interfaces / frames (identical to smac_planner_node)
    _traversability_topic =
      declare_parameter<std::string>("traversability_topic", "traversability");
    _action_name = declare_parameter<std::string>("action_name", "compute_path_to_pose");
    _robot_base_frame = declare_parameter<std::string>("robot_base_frame", "base_link");
    _transform_tolerance = declare_parameter<double>("transform_tolerance", 0.1);

    // Costmap construction from the traversability point cloud (identical to smac_planner_node)
    _traversability_field =
      declare_parameter<std::string>("traversability_field", "traversability");
    _costmap_resolution = declare_parameter<double>("resolution", 0.1);
    _lethal_threshold = declare_parameter<double>("lethal_threshold", 0.65);

    // Minimum robot-centered costmap rectangle. The costmap is always dynamically sized
    // to enclose the incoming cloud's XY extent, but if costmap_width/height are both > 0
    // the bounds are expanded (never shrunk) to also enclose this rectangle, so the
    // costmap never gets smaller than it even if the cloud is sparse/narrow.
    _costmap_width = declare_parameter<double>("costmap_width", 0.0);
    _costmap_height = declare_parameter<double>("costmap_height", 0.0);
    _costmap_center_x = declare_parameter<double>("costmap_center_x", 0.0);
    _costmap_center_y = declare_parameter<double>("costmap_center_y", 0.0);

    // Extra margin (metres) added on every side after the above bounds are computed, so
    // the outermost cloud points/minimum rectangle aren't flush with the costmap edge.
    _costmap_margin_x = declare_parameter<double>("costmap_margin_x", 0.0);
    _costmap_margin_y = declare_parameter<double>("costmap_margin_y", 0.0);

    // Maximum XY distance (metres) from the robot for a cloud point to be included in the
    // costmap at all; farther points are dropped before bounds/cost computation. <= 0 disables.
    _trav_max_dist = declare_parameter<double>("trav_max_dist", 0.0);

    // How far (metres) a start/goal pose outside the costmap may be from the
    // nearest in-bounds cell and still be accepted (clamped to that cell).
    _start_in_bounds_dist = declare_parameter<double>("start_in_bounds_dist", 0.0);
    _goal_in_bounds_dist = declare_parameter<double>("goal_in_bounds_dist", 0.0);

    // Robot radius. Theta* plans for a point; the radius is only enforced if obstacle
    // inflation is enabled (inflation_radius > 0), as the inscribed radius.
    _robot_radius = declare_parameter<double>("robot_radius", 0.3);

    // Obstacle inflation (nav2 InflationLayer cost model), <= 0 disables (default; the
    // costmap is then exactly the one smac_planner_node builds).
    _inflation_radius = declare_parameter<double>("inflation_radius", 0.0);
    _cost_scaling_factor = declare_parameter<double>("cost_scaling_factor", 10.0);

    // General search params (same names/meaning as smac_planner_node)
    _tolerance = declare_parameter<double>("tolerance", 0.25);
    _allow_unknown = declare_parameter<bool>("allow_unknown", true);
    _max_iterations = declare_parameter<int>("max_iterations", 1000000);
    _terminal_checking_interval = declare_parameter<int>("terminal_checking_interval", 5000);
    _max_planning_time = declare_parameter<double>("max_planning_time", 5.0);

    // Theta*-specific params (mirror nav2_theta_star_planner defaults)
    _how_many_corners = declare_parameter<int>("how_many_corners", 8);
    _w_euc_cost = declare_parameter<double>("w_euc_cost", 1.0);
    _w_traversal_cost = declare_parameter<double>("w_traversal_cost", 2.0);
    _use_final_approach_orientation =
      declare_parameter<bool>("use_final_approach_orientation", false);

    // Sanitize
    if (_how_many_corners != 8 && _how_many_corners != 4) {
      RCLCPP_WARN(get_logger(), "how_many_corners must be 4 or 8, overriding to 8.");
      _how_many_corners = 8;
    }
    _lethal_threshold = std::clamp(_lethal_threshold, 1e-3, 1.0);
  }

  ThetaStarParams searchParams() const
  {
    ThetaStarParams p;
    p.w_traversal_cost = _w_traversal_cost;
    p.w_euc_cost = _w_euc_cost;
    p.how_many_corners = _how_many_corners;
    p.allow_unknown = _allow_unknown;
    p.terminal_checking_interval = _terminal_checking_interval;
    p.max_iterations = _max_iterations;
    p.max_planning_time = _max_planning_time;
    p.tolerance_cells = _tolerance / _costmap_resolution;
    return p;
  }

  // Live-tunable search params (as in nav2_theta_star_planner), e.g. via rqt_reconfigure.
  // Costmap-construction params are read once at startup, as in smac_planner_node.
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // Never change params under a running search.
    std::lock_guard<std::mutex> plan_lock(_planning_mutex);
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "how_many_corners") {
        const int v = static_cast<int>(parameter.as_int());
        if (v != 4 && v != 8) {
          result.successful = false;
          result.reason = "how_many_corners must be 4 or 8";
          return result;
        }
        _how_many_corners = v;
      } else if (name == "w_euc_cost") {
        _w_euc_cost = parameter.as_double();
      } else if (name == "w_traversal_cost") {
        _w_traversal_cost = parameter.as_double();
      } else if (name == "allow_unknown") {
        _allow_unknown = parameter.as_bool();
      } else if (name == "terminal_checking_interval") {
        _terminal_checking_interval = static_cast<int>(parameter.as_int());
      } else if (name == "max_iterations") {
        _max_iterations = static_cast<int>(parameter.as_int());
      } else if (name == "max_planning_time") {
        _max_planning_time = parameter.as_double();
      } else if (name == "tolerance") {
        _tolerance = parameter.as_double();
      } else if (name == "use_final_approach_orientation") {
        _use_final_approach_orientation = parameter.as_bool();
      }
    }
    _planner->setParams(searchParams());
    return result;
  }

  // ----------------------------------------------------------------------- //
  // Traversability -> costmap
  // ----------------------------------------------------------------------- //
  void traversabilityCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto costmap = buildCostmap(*msg);
    if (!costmap) {
      return;
    }
    publishCostmap(*costmap, msg->header.frame_id);
    std::lock_guard<std::mutex> lock(_costmap_mutex);
    _costmap = std::move(costmap);
    _global_frame = msg->header.frame_id;
  }

  // Maps raw Costmap2D byte values (0-255) to OccupancyGrid's [0, 100] + -1(unknown)
  // convention, mirroring nav2_costmap_2d::Costmap2DPublisher.
  static const std::array<int8_t, 256> & costTranslationTable()
  {
    static const std::array<int8_t, 256> table = [] {
      std::array<int8_t, 256> t{};
      t[0] = 0;  // no obstacle
      t[nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE] = 99;
      t[nav2_costmap_2d::LETHAL_OBSTACLE] = 100;
      t[nav2_costmap_2d::NO_INFORMATION] = -1;
      // Regular cost values scale the range 1 to 252 (inclusive) to 1 to 98 (inclusive).
      for (int i = 1; i < 253; ++i) {
        t[i] = static_cast<int8_t>(1 + (97 * (i - 1)) / 251);
      }
      return t;
    } ();
    return table;
  }

  void publishCostmap(const nav2_costmap_2d::Costmap2D & costmap, const std::string & frame_id)
  {
    if (_costmap_publisher->get_subscription_count() == 0) {
      return;
    }
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = frame_id;
    grid.info.resolution = static_cast<float>(costmap.getResolution());
    grid.info.width = costmap.getSizeInCellsX();
    grid.info.height = costmap.getSizeInCellsY();

    double wx, wy;
    costmap.mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - costmap.getResolution() / 2.0;
    grid.info.origin.position.y = wy - costmap.getResolution() / 2.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);
    const unsigned char * data = costmap.getCharMap();
    const auto & table = costTranslationTable();
    std::transform(
      data, data + grid.data.size(), grid.data.begin(),
      [&table](unsigned char c) {return table[c];});

    _costmap_publisher->publish(grid);
  }

  unsigned char costFromTraversability(float t) const
  {
    if (!std::isfinite(t)) {
      return nav2_costmap_2d::NO_INFORMATION;
    }
    t = std::clamp(t, 0.0f, 1.0f);
    if (t >= static_cast<float>(_lethal_threshold)) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    const float scaled =
      (t / static_cast<float>(_lethal_threshold)) * ThetaStar::MAX_NON_OBSTACLE_COST;
    return static_cast<unsigned char>(std::lround(scaled));
  }

  // Build a Costmap2D sized to the cloud's XY extent. The cloud is assumed to be a
  // regular square grid with cell spacing == resolution. Returns nullptr on failure.
  // Identical to smac_planner_node, plus optional obstacle inflation at the end.
  std::shared_ptr<nav2_costmap_2d::Costmap2D> buildCostmap(
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    bool has_x = false, has_y = false, has_t = false;
    for (const auto & f : cloud.fields) {
      has_x |= (f.name == "x");
      has_y |= (f.name == "y");
      has_t |= (f.name == _traversability_field);
    }
    if (!has_x || !has_y || !has_t) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Traversability cloud missing required fields (need x, y, %s).",
        _traversability_field.c_str());
      return nullptr;
    }

    const size_t n = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    if (n == 0) {
      return nullptr;
    }

    // If limiting cloud points by distance from the robot, or enclosing a minimum
    // robot-centered rectangle, the robot's pose in the cloud's frame is needed up front.
    const bool need_robot_pose = _trav_max_dist > 0.0 ||
      (_costmap_width > 0.0 && _costmap_height > 0.0);
    geometry_msgs::msg::PoseStamped robot_pose;
    if (need_robot_pose && !getRobotPose(cloud.header.frame_id, robot_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not look up robot pose in '%s'; skipping costmap update.",
        cloud.header.frame_id.c_str());
      return nullptr;
    }
    const double robot_x = robot_pose.pose.position.x;
    const double robot_y = robot_pose.pose.position.y;

    // First pass: XY bounds of valid (and, if trav_max_dist > 0, in-range) points.
    sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    bool any = false;
    for (; it_x != it_x.end(); ++it_x, ++it_y) {
      const float x = *it_x;
      const float y = *it_y;
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      if (_trav_max_dist > 0.0 && std::hypot(x - robot_x, y - robot_y) > _trav_max_dist) {
        continue;
      }
      min_x = std::min(min_x, static_cast<double>(x));
      min_y = std::min(min_y, static_cast<double>(y));
      max_x = std::max(max_x, static_cast<double>(x));
      max_y = std::max(max_y, static_cast<double>(y));
      any = true;
    }
    if (!any) {
      return nullptr;
    }

    // Expand the bounds to enclose a minimum robot-centered rectangle, if configured
    // (costmap_width/height <= 0 disables this and leaves the cloud-only bounds above).
    if (_costmap_width > 0.0 && _costmap_height > 0.0) {
      // costmap_center_x/y are offsets in the robot's own frame (e.g. positive x shifts
      // the minimum rectangle ahead of the robot), rotated into the cloud's (global) frame.
      const double yaw = tf2::getYaw(robot_pose.pose.orientation);
      const double center_x = robot_pose.pose.position.x +
        std::cos(yaw) * _costmap_center_x - std::sin(yaw) * _costmap_center_y;
      const double center_y = robot_pose.pose.position.y +
        std::sin(yaw) * _costmap_center_x + std::cos(yaw) * _costmap_center_y;
      min_x = std::min(min_x, center_x - _costmap_width / 2.0);
      max_x = std::max(max_x, center_x + _costmap_width / 2.0);
      min_y = std::min(min_y, center_y - _costmap_height / 2.0);
      max_y = std::max(max_y, center_y + _costmap_height / 2.0);
    }

    // Pad every side so the outermost points/rectangle aren't flush with the costmap edge.
    min_x -= _costmap_margin_x;
    max_x += _costmap_margin_x;
    min_y -= _costmap_margin_y;
    max_y += _costmap_margin_y;

    const double res = _costmap_resolution;
    const unsigned int size_x =
      static_cast<unsigned int>(std::lround((max_x - min_x) / res)) + 1u;
    const unsigned int size_y =
      static_cast<unsigned int>(std::lround((max_y - min_y) / res)) + 1u;
    // Origin is the lower-left cell corner; point at (min_x, min_y) lands in cell (0, 0)'s
    // center, i.e. cell center == origin + (i + 0.5) * res == min + i * res.
    const double origin_x = min_x - 0.5 * res;
    const double origin_y = min_y - 0.5 * res;

    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
      size_x, size_y, res, origin_x, origin_y, nav2_costmap_2d::NO_INFORMATION);

    // Second pass: stamp costs.
    sensor_msgs::PointCloud2ConstIterator<float> px(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> py(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> pt(cloud, _traversability_field);
    for (; px != px.end(); ++px, ++py, ++pt) {
      const float x = *px;
      const float y = *py;
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      if (_trav_max_dist > 0.0 && std::hypot(x - robot_x, y - robot_y) > _trav_max_dist) {
        continue;
      }
      unsigned int mx, my;
      if (!costmap->worldToMap(static_cast<double>(x), static_cast<double>(y), mx, my)) {
        continue;
      }
      costmap->setCost(mx, my, costFromTraversability(*pt));
    }

    if (_inflation_radius > 0.0) {
      inflateCostmap(*costmap, _robot_radius, _inflation_radius, _cost_scaling_factor);
    }

    return costmap;
  }

  // ----------------------------------------------------------------------- //
  // Action server
  // ----------------------------------------------------------------------- //
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ComputePathToPose::Goal>)
  {
    std::lock_guard<std::mutex> lock(_costmap_mutex);
    if (!_costmap) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: no traversability map received yet.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Run planning off the executor thread; serialized via _planning_mutex.
    std::thread{std::bind(&ThetaStarPlannerNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> plan_lock(_planning_mutex);
    auto result = std::make_shared<ComputePathToPose::Result>();

    auto goal = *goal_handle->get_goal();
    goal.goal.header.stamp = this->now();
    const auto t0 = std::chrono::steady_clock::now();

    // Snapshot the latest costmap and its frame.
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    std::string global_frame;
    {
      std::lock_guard<std::mutex> lock(_costmap_mutex);
      costmap = _costmap;
      global_frame = _global_frame;
    }
    if (!costmap) {
      result->error_code = ComputePathToPose::Result::UNKNOWN;
      result->error_msg = "No traversability map available";
      goal_handle->abort(result);
      return;
    }

    // Resolve goal and start poses in the costmap (global) frame.
    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped start_pose;
    if (!transformToGlobal(goal.goal, global_frame, goal_pose)) {
      result->error_code = ComputePathToPose::Result::TF_ERROR;
      result->error_msg = "Could not transform goal into '" + global_frame + "'";
      goal_handle->abort(result);
      return;
    }
    if (goal.use_start) {
      if (!transformToGlobal(goal.start, global_frame, start_pose)) {
        result->error_code = ComputePathToPose::Result::TF_ERROR;
        result->error_msg = "Could not transform start into '" + global_frame + "'";
        goal_handle->abort(result);
        return;
      }
    } else if (!getRobotPose(global_frame, start_pose)) {
      result->error_code = ComputePathToPose::Result::TF_ERROR;
      result->error_msg = "Could not look up robot pose in '" + global_frame + "'";
      goal_handle->abort(result);
      return;
    }

    _planning_start_publisher->publish(start_pose);
    _planning_goal_publisher->publish(goal_pose);

    auto cancel_checker = [goal_handle]() {return goal_handle->is_canceling();};

    // "respect_heading" in planner_id forces the exact goal orientation on the final pose,
    // overriding use_final_approach_orientation for this request only (same convention as
    // smac_planner_node, where it overrides goal_heading_mode).
    const bool respect_heading = goal.planner_id.find("respect_heading") != std::string::npos;

    nav_msgs::msg::Path path;
    uint16_t error_code = ComputePathToPose::Result::NONE;
    std::string error_msg;
    const bool ok = createPlan(
      start_pose, goal_pose, costmap, global_frame, cancel_checker, path, error_code, error_msg,
      respect_heading);

    if (goal_handle->is_canceling()) {
      result->error_code = ComputePathToPose::Result::UNKNOWN;
      result->error_msg = "Planning canceled";
      goal_handle->canceled(result);
      return;
    }

    if (!ok) {
      result->error_code = error_code;
      result->error_msg = error_msg;
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "Planning failed: %s", error_msg.c_str());
      return;
    }

    result->path = path;
    const auto t1 = std::chrono::steady_clock::now();
    const double planning_time_s = std::chrono::duration<double>(t1 - t0).count();
    result->planning_time = rclcpp::Duration::from_seconds(planning_time_s);
    result->error_code = ComputePathToPose::Result::NONE;

    if (!path.poses.empty()) {
      const auto & reached = path.poses.back().pose.position;
      const double goal_error = std::hypot(
        reached.x - goal_pose.pose.position.x, reached.y - goal_pose.pose.position.y);
      RCLCPP_INFO(
        get_logger(), "Planning took %.4f s, goal distance error %.3f m.",
        planning_time_s, goal_error);
    } else {
      RCLCPP_INFO(
        get_logger(), "Planning took %.4f s, empty path (no goal reached).", planning_time_s);
    }

    if (_plan_publisher->get_subscription_count() > 0) {
      _plan_publisher->publish(path);
    }
    goal_handle->succeed(result);
  }

  // Converts (wx, wy) to a map cell. If it's outside the costmap, clamps it to the nearest
  // valid cell via worldToMapEnforceBounds and accepts it only if that cell's center is
  // within max_dist metres of the original point. `in_bounds` reports which case applied.
  bool toMapCell(
    const nav2_costmap_2d::Costmap2D & costmap,
    double wx, double wy, double max_dist,
    CoordsM & cell, bool & in_bounds) const
  {
    unsigned int umx, umy;
    if (costmap.worldToMap(wx, wy, umx, umy)) {
      cell = {static_cast<int>(umx), static_cast<int>(umy)};
      in_bounds = true;
      return true;
    }
    in_bounds = false;
    int imx, imy;
    costmap.worldToMapEnforceBounds(wx, wy, imx, imy);
    double cwx, cwy;
    costmap.mapToWorld(static_cast<unsigned int>(imx), static_cast<unsigned int>(imy), cwx, cwy);
    if (std::hypot(cwx - wx, cwy - wy) > max_dist) {
      return false;
    }
    cell = {imx, imy};
    return true;
  }

  // ----------------------------------------------------------------------- //
  // Planning core (mirrors ThetaStarPlanner::createPlan / getPlan)
  // ----------------------------------------------------------------------- //
  bool createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap,
    const std::string & global_frame,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan,
    uint16_t & error_code,
    std::string & error_msg,
    bool respect_heading)
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    // Setup output message.
    plan.header.stamp = now();
    plan.header.frame_id = global_frame;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.orientation.w = 1.0;

    CoordsM start_cell{}, goal_cell{};
    bool start_in_bounds = false, goal_in_bounds = false;
    if (!toMapCell(
        *costmap, start.pose.position.x, start.pose.position.y, _start_in_bounds_dist,
        start_cell, start_in_bounds))
    {
      error_code = ComputePathToPose::Result::START_OUTSIDE_MAP;
      error_msg = "Start outside costmap bounds (beyond start_in_bounds_dist)";
      return false;
    }
    if (!toMapCell(
        *costmap, goal.pose.position.x, goal.pose.position.y, _goal_in_bounds_dist,
        goal_cell, goal_in_bounds))
    {
      error_code = ComputePathToPose::Result::GOAL_OUTSIDE_MAP;
      error_msg = "Goal outside costmap bounds (beyond goal_in_bounds_dist)";
      return false;
    }

    // Corner case: start and goal in the same cell (as in ThetaStarPlanner::createPlan).
    if (start_cell.x == goal_cell.x && start_cell.y == goal_cell.y) {
      pose.pose = start.pose;
      // Unless use_final_approach_orientation (start orientation, to avoid movement from
      // the controller), the single pose takes the goal orientation.
      if (!_use_final_approach_orientation || respect_heading) {
        pose.pose.orientation = goal.pose.orientation;
      }
      plan.poses.push_back(pose);
      return true;
    }

    // The start cell is treated as free (upstream clears it in the costmap), so only the
    // goal can be unsafe. Upstream reports LETHAL as GoalOccupied and inscribed/unknown as a
    // generic exception; both are GOAL_OCCUPIED here.
    _planner->setCostmap(costmap.get());
    _planner->setStartAndGoal(start_cell, goal_cell);
    if (!_planner->isSafe(goal_cell.x, goal_cell.y)) {
      error_code = ComputePathToPose::Result::GOAL_OCCUPIED;
      error_msg = "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
        std::to_string(goal.pose.position.y) + ") was in lethal cost";
      return false;
    }

    std::vector<CoordsW> raw_path;
    const auto search_t0 = std::chrono::steady_clock::now();
    const SearchStatus status = _planner->generatePath(raw_path, cancel_checker);
    const double search_time_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - search_t0).count();
    RCLCPP_INFO(
      get_logger(), "ThetaStar::generatePath took %.4f s (%d nodes opened).",
      search_time_s, _planner->nodesOpened());

    switch (status) {
      case SearchStatus::GOAL_REACHED:
      case SearchStatus::WITHIN_TOLERANCE:
        break;
      case SearchStatus::CANCELED:
        error_code = ComputePathToPose::Result::UNKNOWN;
        error_msg = "Planning canceled";
        return false;
      case SearchStatus::TIMEOUT:
        error_code = ComputePathToPose::Result::TIMEOUT;
        error_msg = "Exceeded maximum iterations (" + std::to_string(_max_iterations) +
          ") or planning time (" + std::to_string(_max_planning_time) + " s) after " +
          std::to_string(_planner->nodesOpened()) + " nodes, nothing within tolerance " +
          std::to_string(_tolerance) + " m";
        return false;
      case SearchStatus::NO_PATH:
      default:
        error_code = ComputePathToPose::Result::NO_VALID_PATH;
        error_msg = "No valid path found: search exhausted all reachable cells after " +
          std::to_string(_planner->nodesOpened()) + " nodes, nothing within tolerance " +
          std::to_string(_tolerance) + " m of the goal";
        return false;
    }

    // Path vertices are cell centers; snap the endpoints to the exact requested poses where
    // that's meaningful (start/goal inside the map, goal actually reached) and the moved
    // first/last segment still only crosses safe cells.
    if (raw_path.size() >= 2) {
      const CoordsW exact_start{start.pose.position.x, start.pose.position.y};
      if (start_in_bounds && segmentSafe(*costmap, exact_start, raw_path[1])) {
        raw_path.front() = exact_start;
      }
      const CoordsW exact_goal{goal.pose.position.x, goal.pose.position.y};
      if (status == SearchStatus::GOAL_REACHED && goal_in_bounds &&
        segmentSafe(*costmap, raw_path[raw_path.size() - 2], exact_goal))
      {
        raw_path.back() = exact_goal;
      }
    }

    if (_raw_plan_publisher->get_subscription_count() > 0) {
      nav_msgs::msg::Path raw_plan;
      raw_plan.header = plan.header;
      fillPath(raw_path, goal, respect_heading, raw_plan);
      _raw_plan_publisher->publish(raw_plan);
    }

    fillPath(
      ThetaStar::linearInterpolation(raw_path, costmap->getResolution()),
      goal, respect_heading, plan);
    return true;
  }

  // Whether every cell the segment a-b passes through (sampled at res/4) is in the costmap
  // and safe for the planner (the start cell counts as safe, as in the search).
  bool segmentSafe(
    const nav2_costmap_2d::Costmap2D & costmap, const CoordsW & a, const CoordsW & b) const
  {
    const double step = costmap.getResolution() / 4.0;
    const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / step)));
    for (int k = 0; k <= n; ++k) {
      const double s = static_cast<double>(k) / n;
      unsigned int mx, my;
      if (!costmap.worldToMap(a.x + s * (b.x - a.x), a.y + s * (b.y - a.y), mx, my) ||
        !_planner->isSafe(static_cast<int>(mx), static_cast<int>(my)))
      {
        return false;
      }
    }
    return true;
  }

  // Fills `plan.poses` from world points (start to end). Every pose faces the next one;
  // the final pose takes the goal orientation, or (use_final_approach_orientation, unless
  // respect_heading) the direction of the last segment.
  void fillPath(
    const std::vector<CoordsW> & points,
    const geometry_msgs::msg::PoseStamped & goal,
    bool respect_heading,
    nav_msgs::msg::Path & plan) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    plan.poses.clear();
    plan.poses.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      pose.pose.position.z = 0.0;
      if (i + 1 < points.size()) {
        pose.pose.orientation = yawToQuaternion(
          std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x));
      } else if (_use_final_approach_orientation && !respect_heading && points.size() > 1) {
        pose.pose.orientation = plan.poses.back().pose.orientation;
      } else {
        pose.pose.orientation = goal.pose.orientation;
      }
      plan.poses.push_back(pose);
    }
  }

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

  // ----------------------------------------------------------------------- //
  // TF helpers
  // ----------------------------------------------------------------------- //
  bool transformToGlobal(
    const geometry_msgs::msg::PoseStamped & in,
    const std::string & global_frame,
    geometry_msgs::msg::PoseStamped & out)
  {
    if (in.header.frame_id.empty() || in.header.frame_id == global_frame) {
      out = in;
      out.header.frame_id = global_frame;
      return true;
    }
    try {
      out = _tf_buffer->transform(
        in, global_frame, tf2::durationFromSec(_transform_tolerance));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF transform to '%s' failed: %s", global_frame.c_str(), ex.what());
      return false;
    }
    return true;
  }

  bool getRobotPose(const std::string & global_frame, geometry_msgs::msg::PoseStamped & pose)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = _tf_buffer->lookupTransform(
        global_frame, _robot_base_frame, tf2::TimePointZero,
        tf2::durationFromSec(_transform_tolerance));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Robot pose lookup failed: %s", ex.what());
      return false;
    }
    pose.header.frame_id = global_frame;
    pose.header.stamp = tf.header.stamp;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return true;
  }

  // ----------------------------------------------------------------------- //
  // Members
  // ----------------------------------------------------------------------- //
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _traversability_sub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _plan_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _raw_plan_publisher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _planning_start_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _planning_goal_publisher;
  rclcpp_action::Server<ComputePathToPose>::SharedPtr _action_server;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

  // Planner
  std::unique_ptr<ThetaStar> _planner;

  // Costmap (rebuilt on every traversability message; only the latest is kept)
  std::shared_ptr<nav2_costmap_2d::Costmap2D> _costmap;
  std::string _global_frame;
  std::mutex _costmap_mutex;
  std::mutex _planning_mutex;

  // Parameters
  std::string _traversability_topic, _action_name, _robot_base_frame, _traversability_field;
  double _transform_tolerance;
  double _costmap_resolution;
  double _lethal_threshold;
  double _costmap_width;
  double _costmap_height;
  double _costmap_center_x;
  double _costmap_center_y;
  double _costmap_margin_x;
  double _costmap_margin_y;
  double _trav_max_dist;
  double _start_in_bounds_dist;
  double _goal_in_bounds_dist;
  double _robot_radius;
  double _inflation_radius;
  double _cost_scaling_factor;
  double _tolerance;
  bool _allow_unknown;
  int _max_iterations;
  int _terminal_checking_interval;
  double _max_planning_time;
  int _how_many_corners;
  double _w_euc_cost;
  double _w_traversal_cost;
  bool _use_final_approach_orientation;
};

}  // namespace theta_star_planner_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<theta_star_planner_node::ThetaStarPlannerNode>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
