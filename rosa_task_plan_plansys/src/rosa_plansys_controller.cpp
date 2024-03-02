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
#include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rosa_task_plan_plansys
{
  RosaPlansysControllerNode::RosaPlansysControllerNode(const std::string & node_name)
  : rclcpp::Node(node_name)
  {
    this->declare_parameter("rosa_actions", rclcpp::PARAMETER_STRING_ARRAY);

    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();

    selectable_actions_client_ =
      this->create_client<rosa_msgs::srv::SelectableActions>("/rosa_kb/action/selectable");

    update_actions_feasibility();

    rosa_events_sub_ = create_subscription<std_msgs::msg::String>(
      "/rosa_kb/events",
      10,
      std::bind(&RosaPlansysControllerNode::rosa_events_cb, this, _1));
  }

  void RosaPlansysControllerNode::rosa_events_cb(const std_msgs::msg::String msg){
    if (msg.data == "insert_monitoring_data" || msg.data == "action_update"){
      update_actions_feasibility();
    }
  }

  void RosaPlansysControllerNode::update_actions_feasibility(){
    auto rosa_actions = this->get_parameter("rosa_actions").as_string_array();

    while (!selectable_actions_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "/rosa_kb/action/selectable service not available, waiting again...");
    }
    auto request = std::make_shared<rosa_msgs::srv::SelectableActions::Request>();
    auto response = selectable_actions_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto selectable_actions = response.get()->actions;
      for (auto action: rosa_actions){
        if(std::any_of(selectable_actions.begin(), selectable_actions.end(),
          [&](const rosa_msgs::msg::Action& elem) { return elem.name.c_str() == action; })) {
          std::string action_predicate = "("+action+"_feasible)";
          problem_expert_->addPredicate(plansys2::Predicate(action_predicate));
        } else{
          std::string action_predicate = "("+action+"_feasible)";
          problem_expert_->removePredicate(plansys2::Predicate(action_predicate));
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service actions selectable");
    }
    return;
  }

}
