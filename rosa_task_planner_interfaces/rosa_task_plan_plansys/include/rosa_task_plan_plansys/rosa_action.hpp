// Copyright 2024 Gustavo Rezende Silva
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
#ifndef ROSA_TASK_PLAN_PLANSYS__ROSA_ACTION_HPP_
#define ROSA_TASK_PLAN_PLANSYS__ROSA_ACTION_HPP_

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosa_msgs/srv/action_query.hpp"

// #include "rosa_task_plan_plansys/visibility_control.h"

namespace rosa_task_plan_plansys
{

  class RosaAction : public plansys2::ActionExecutorClient
  {
  public:
    RosaAction(const std::string & node_name,
      const std::chrono::nanoseconds & rate);

    virtual ~RosaAction();

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state);

  private:
    std::string action_name_;

    rclcpp::CallbackGroup::SharedPtr callback_group_actions_client_;
    rclcpp::Client<rosa_msgs::srv::ActionQuery>::SharedPtr action_req_client_;
  };

}  // namespace rosa_task_plan_plansys

#endif  // ROSA_TASK_PLAN_PLANSYS__ROSA_ACTION_HPP_
