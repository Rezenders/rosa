// Copyright 2023 Gustavo Rezende Silva
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

#ifndef ROSA_PLAN__IS_TASK_FEASIBLE_HPP_
#define ROSA_PLAN__IS_TASK_FEASIBLE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rosa_msgs/srv/tasks_matched.hpp"

namespace rosa_plan
{

class IsTaskFeasible : public BT::ConditionNode
{
public:
  explicit IsTaskFeasible(const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("task_name"),
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rosa_msgs::srv::TasksMatched>::SharedPtr selectable_tasks_client;
};

} //namespace rosa_plan

#endif  // ROSA_PLAN__IS_TASK_FEASIBLE_HPP_
