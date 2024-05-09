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

#include "rclcpp/rclcpp.hpp"
#include "rosa_msgs/srv/action_query_array.hpp"
#include "rosa_msgs/msg/action.hpp"
#include "std_msgs/msg/string.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rosa_task_plan_plansys/visibility_control.h"

namespace rosa_task_plan_plansys
{

class RosaPlansysController : public rclcpp::Node
{
public:
  RosaPlansysController(const std::string & node_name);

  virtual ~RosaPlansysController();

protected:
  rclcpp::CallbackGroup::SharedPtr step_timer_cb_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rosa_events_sub_;
  void rosa_events_cb(const std_msgs::msg::String msg);

  rclcpp::CallbackGroup::SharedPtr callback_group_actions_client_;
  rclcpp::Client<rosa_msgs::srv::ActionQueryArray>::SharedPtr selectable_actions_client_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  bool first_iteration_ = true;
  void execute_plan();
  bool update_actions_feasibility();

  void step();
  virtual void finish_controlling();
};

}  // namespace rosa_task_plan_plansys

#endif  // ROSA_TASK_PLAN_PLANSYS__ROSA_PLANSYS_CONTROLLER_HPP_
