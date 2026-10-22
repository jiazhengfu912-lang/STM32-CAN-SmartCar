// Copyright 2026 SmartCar contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"

namespace smartcar_nav2
{
// Humble tf2 0.25.23 的异步请求与 TF 到达回调有锁序反转；不更改障碍层或观测时间。
class SafeCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  // 保留 Nav2 原有代价图名称、命名空间与参数解析。
  SafeCostmap(const std::string & name, const std::string & parent)
  : Costmap2DROS(name, parent, name) {}

protected:
  // TF、MessageFilter 与其超时定时器共用原代价图互斥回调组，消除并发锁序反转。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = Costmap2DROS::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) {
      return result;
    }
    // 此阶段障碍订阅尚未激活，可安全替换原来使用独立线程的监听器。
    tf_listener_.reset();
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, shared_from_this(), false, tf2_ros::DynamicListenerQoS(),
      tf2_ros::StaticListenerQoS(), options, options);
    // 回调组仍在代价图专用执行线程运行，控制器线程可继续使用有界 TF 查询。
    tf_buffer_->setUsingDedicatedThread(true);
    RCLCPP_INFO(get_logger(), "SmartCar: TF and scan filters share one callback executor");
    return result;
  }

  // 先等待专用回调执行完，再析构过滤器、TF 和图层；不与正在运行的回调并发销毁。
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    executor_thread_.reset();
    return Costmap2DROS::on_cleanup(state);
  }
};
}  // namespace smartcar_nav2
