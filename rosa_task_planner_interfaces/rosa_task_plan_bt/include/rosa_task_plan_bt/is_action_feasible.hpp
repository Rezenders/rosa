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

#ifndef ROSA_TASK_PLAN_BT__IS_ACTION_FEASIBLE_HPP_
#define ROSA_TASK_PLAN_BT__IS_ACTION_FEASIBLE_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rosa_msgs/srv/action_query_array.hpp"

using namespace std::chrono_literals;

namespace rosa_task_plan_bt
{

template<class T>
class IsActionFeasible : public BT::ConditionNode
{
public:
  explicit IsActionFeasible(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : BT::ConditionNode(xml_tag_name, conf)
  {
    node_ = config().blackboard->get<T>("node");

    selectable_actions_client_ =
      this->node_->template create_client<rosa_msgs::srv::ActionQueryArray>("/rosa_kb/action/selectable");
  };

  BT::NodeStatus tick() override {
    std::string action_name;
    getInput("action_name", action_name);

    auto request = std::make_shared<rosa_msgs::srv::ActionQueryArray::Request>();

    while (!selectable_actions_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->node_->template get_logger(), "Interrupted while waiting for the service. Exiting.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(this->node_->template get_logger(), "/rosa_kb/action/selectable service not available, waiting again...");
    }

    auto response = selectable_actions_client_->async_send_request(request);
    if (response.wait_for(1s) == std::future_status::ready)
    {
      auto selectable_actions = response.get()->actions;
      for (auto action: selectable_actions){
        if (std::strcmp(action.name.c_str(), action_name.c_str()) == 0){
          return BT::NodeStatus::SUCCESS;
        }
      }
      return BT::NodeStatus::FAILURE;
    } else {
      RCLCPP_ERROR(this->node_->template get_logger(), "Failed to call service actions selectable");
      return BT::NodeStatus::FAILURE;
    }
  };

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("action_name"),
      });
  }

private:
  T node_;
  rclcpp::Client<rosa_msgs::srv::ActionQueryArray>::SharedPtr selectable_actions_client_;
};

} //namespace rosa_task_plan_bt

#endif  // ROSA_TASK_PLAN_BT__IS_ACTION_FEASIBLE_HPP_
