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
#ifndef ROSA_TASK_PLAN_PLANSYS__ROSA_PLANSYS_CONTROLLER_HPP_
#define ROSA_TASK_PLAN_PLANSYS__ROSA_PLANSYS_CONTROLLER_HPP_

#include <algorithm>

#include "rosa_task_plan_plansys/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "rosa_msgs/srv/selectable_actions.hpp"
#include "rosa_msgs/msg/action.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace rosa_task_plan_plansys
{

class RosaPlansysControllerNode : public rclcpp::Node
{
public:
  RosaPlansysControllerNode(const std::string & node_name);

  virtual ~RosaPlansysControllerNode();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rosa_events_sub_;
  rclcpp::Client<rosa_msgs::srv::SelectableActions>::SharedPtr selectable_actions_client_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;

  void rosa_events_cb(const std_msgs::msg::String msg);
  void update_actions_feasibility();
};

}  // namespace rosa_task_plan_plansys

#endif  // ROSA_TASK_PLAN_PLANSYS__ROSA_PLANSYS_CONTROLLER_HPP_
