// Copyright 2026 SmartCar contributors
// Copyright (c) 2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
// FollowPath 循环沿用 Navigation2 1.1.20 的调度与插件接口（Intel 等，Apache-2.0）。
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include "nav2_controller/controller_server.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "safe_costmap.hpp"

namespace smartcar_nav2
{
class SafeControllerServer : public nav2_controller::ControllerServer
{
public:
  // 只替换代价图执行策略；跟踪器、碰撞检查、速度边界和参数仍来自原 Nav2。
  SafeControllerServer()
  {
    costmap_ros_ = std::make_shared<SafeCostmap>("local_costmap", get_namespace());
  }

protected:
  // 按 Nav2 1.1.20 初始化插件；直接绑定安全循环，避免创建后立刻销毁后台动作线程。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    auto node = shared_from_this();
    goal_checker_ids_concat_.clear();
    controller_ids_concat_.clear();

    RCLCPP_INFO(get_logger(), "Configuring controller interface");

    get_parameter("progress_checker_plugin", progress_checker_id_);
    if (progress_checker_id_ == default_progress_checker_id_) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_progress_checker_id_ + ".plugin",
        rclcpp::ParameterValue(default_progress_checker_type_));
    }

    RCLCPP_INFO(get_logger(), "getting goal checker plugins..");
    get_parameter("goal_checker_plugins", goal_checker_ids_);
    if (goal_checker_ids_ == default_goal_checker_ids_) {
      for (size_t i = 0; i < default_goal_checker_ids_.size(); ++i) {
        nav2_util::declare_parameter_if_not_declared(
          node, default_goal_checker_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_goal_checker_types_[i]));
      }
    }

    get_parameter("controller_plugins", controller_ids_);
    if (controller_ids_ == default_ids_) {
      for (size_t i = 0; i < default_ids_.size(); ++i) {
        nav2_util::declare_parameter_if_not_declared(
          node, default_ids_[i] + ".plugin",
          rclcpp::ParameterValue(default_types_[i]));
      }
    }

    controller_types_.resize(controller_ids_.size());
    goal_checker_types_.resize(goal_checker_ids_.size());

    get_parameter("controller_frequency", controller_frequency_);
    get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
    get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
    get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
    RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

    std::string speed_limit_topic;
    get_parameter("speed_limit_topic", speed_limit_topic);
    get_parameter("failure_tolerance", failure_tolerance_);
    get_parameter("publish_zero_velocity", publish_zero_velocity_);

    costmap_ros_->configure();
    // Launch a thread to run the costmap node
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    try {
      progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
      progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
      RCLCPP_INFO(
        get_logger(), "Created progress_checker : %s of type %s",
        progress_checker_id_.c_str(), progress_checker_type_.c_str());
      progress_checker_->initialize(node, progress_checker_id_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create progress_checker. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }

    for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
      try {
        goal_checker_types_[i] = nav2_util::get_plugin_type_param(node, goal_checker_ids_[i]);
        nav2_core::GoalChecker::Ptr goal_checker =
          goal_checker_loader_.createUniqueInstance(goal_checker_types_[i]);
        RCLCPP_INFO(
          get_logger(), "Created goal checker : %s of type %s",
          goal_checker_ids_[i].c_str(), goal_checker_types_[i].c_str());
        goal_checker->initialize(node, goal_checker_ids_[i], costmap_ros_);
        goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          get_logger(),
          "Failed to create goal checker. Exception: %s", ex.what());
        return nav2_util::CallbackReturn::FAILURE;
      }
    }

    for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
      goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
    }

    RCLCPP_INFO(
      get_logger(),
      "Controller Server has %s goal checkers available.", goal_checker_ids_concat_.c_str());

    for (size_t i = 0; i != controller_ids_.size(); i++) {
      try {
        controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
        nav2_core::Controller::Ptr controller =
          lp_loader_.createUniqueInstance(controller_types_[i]);
        RCLCPP_INFO(
          get_logger(), "Created controller : %s of type %s",
          controller_ids_[i].c_str(), controller_types_[i].c_str());
        controller->configure(
          node, controller_ids_[i],
          costmap_ros_->getTfBuffer(), costmap_ros_);
        controllers_.insert({controller_ids_[i], controller});
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          get_logger(),
          "Failed to create controller. Exception: %s", ex.what());
        return nav2_util::CallbackReturn::FAILURE;
      }
    }

    for (size_t i = 0; i != controller_ids_.size(); i++) {
      controller_ids_concat_ += controller_ids_[i] + std::string(" ");
    }

    RCLCPP_INFO(
      get_logger(),
      "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

    odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
    vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    // Create the action server that we implement with our followPath method
    action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "follow_path",
      std::bind(&SafeControllerServer::computeSafeControl, this),
      nullptr,
      std::chrono::milliseconds(500),
      true);

    // Set subscribtion to the speed limiting topic
    speed_limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
      speed_limit_topic, rclcpp::QoS(10),
      [this](const nav2_msgs::msg::SpeedLimit::SharedPtr msg) {
        for (auto & entry : controllers_) {
          entry.second->setSpeedLimit(msg->speed_limit, msg->percentage);
        }
      });

    return nav2_util::CallbackReturn::SUCCESS;
  }

  // 变换失败立即退出本次跟踪，绝不把未初始化的零位姿传给目标检查器。
  geometry_msgs::msg::PoseStamped checkedGoalPose()
  {
    geometry_msgs::msg::PoseStamped goal;
    auto tolerance = rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance());
    if (!nav_2d_utils::transformPose(
        costmap_ros_->getTfBuffer(), costmap_ros_->getGlobalFrameID(), end_pose_, goal, tolerance))
    {
      throw nav2_core::PlannerException("Goal transform unavailable: refusing goal completion");
    }
    return goal;
  }

  // 仅用成功转换的目标与新鲜车辆位姿做原有容差判断，不扩大到达范围。
  bool checkedGoalReached()
  {
    const auto goal = checkedGoalPose();
    geometry_msgs::msg::PoseStamped pose;
    if (!getRobotPose(pose)) {
      throw nav2_core::PlannerException("Robot transform unavailable: refusing goal completion");
    }
    const auto velocity = nav_2d_utils::twist2Dto3D(getThresholdedTwist(odom_sub_->getTwist()));
    return goal_checkers_[current_goal_checker_]->isGoalReached(pose.pose, goal.pose, velocity);
  }

  // 保留原控制循环、插件选择、抢占、进度与碰撞检查；所有退出路径均停车。
  void computeSafeControl()
  {
    std::lock_guard<std::mutex> lock(dynamic_params_lock_);
    RCLCPP_INFO(get_logger(), "Received FollowPath goal with checked TF completion");
    try {
      const auto request = action_server_->get_current_goal();
      if (!findControllerId(request->controller_id, current_controller_) ||
        !findGoalCheckerId(request->goal_checker_id, current_goal_checker_))
      {
        publishZeroVelocity();
        action_server_->terminate_current();
        return;
      }
      setPlannerPath(request->path);
      progress_checker_->reset();
      last_valid_cmd_time_ = now();
      rclcpp::WallRate rate(controller_frequency_);
      while (rclcpp::ok()) {
        if (!action_server_->is_server_active()) {
          publishZeroVelocity();
          return;
        }
        if (action_server_->is_cancel_requested()) {
          publishZeroVelocity();
          action_server_->terminate_all();
          return;
        }
        // 等待观测时也响应取消/生命周期停用，避免 Reset 等待一个永不退出的动作。
        if (!costmap_ros_->isCurrent()) {
          publishZeroVelocity();
          rate.sleep();
          continue;
        }
        updateGlobalPath();
        // 发出速度前先验证目标 TF，失败时本轮不产生非零速度。
        checkedGoalPose();
        computeAndPublishVelocity();
        if (checkedGoalReached()) {
          publishZeroVelocity();
          RCLCPP_INFO(get_logger(), "Reached goal with valid TF");
          action_server_->succeeded_current();
          return;
        }
        if (!rate.sleep()) {
          RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz", controller_frequency_);
        }
      }
      publishZeroVelocity();
      action_server_->terminate_current();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
      publishZeroVelocity();
      action_server_->terminate_current();
    }
  }
};
}  // namespace smartcar_nav2

// 独立可执行文件随工作空间构建，不覆盖系统安装的 Nav2 或 tf2。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_nav2::SafeControllerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
