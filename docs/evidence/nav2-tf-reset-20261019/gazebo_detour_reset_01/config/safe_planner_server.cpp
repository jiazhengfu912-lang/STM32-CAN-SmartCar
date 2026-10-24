// Copyright 2026 SmartCar contributors
// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include "nav2_planner/planner_server.hpp"
#include "safe_costmap.hpp"

namespace smartcar_nav2
{
class SafePlannerServer : public nav2_planner::PlannerServer
{
public:
  // 全局代价图也使用相同回调串行策略，规划算法和障碍记忆保持原配置。
  SafePlannerServer()
  {
    costmap_ros_ = std::make_shared<SafeCostmap>("global_costmap", get_namespace());
  }
};
}  // namespace smartcar_nav2

// 保留标准 planner_server 节点名与 ComputePath 动作接口。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_nav2::SafePlannerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
