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
#include "rosa_task_plan_plansys/rosa_action.hpp"

using namespace std::chrono_literals;
namespace rosa_task_plan_plansys
{

  RosaAction::RosaAction(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  :   plansys2::ActionExecutorClient(node_name, rate)
  {
    callback_group_actions_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    action_req_client_ =
      this->create_client<rosa_msgs::srv::ActionQuery>(
        "/rosa_kb/action/request",
        rmw_qos_profile_services_default,
        callback_group_actions_client_);
    action_name_ = this->get_parameter("action_name").as_string();
  }

  RosaAction::~RosaAction()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  RosaAction::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Action requested to ROSA: %s", action_name_.c_str());

    while (!action_req_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(this->get_logger(), "service /rosa_kb/action/request not available, waiting again...");
    }

    auto request = std::make_shared<rosa_msgs::srv::ActionQuery::Request>();
    request->action.name = action_name_;
    request->action.is_required = true;

    auto response = action_req_client_->async_send_request(request);
    auto response_status = response.wait_for(1s);
    if (response_status == std::future_status::ready && response.get()->success == true)
    {
      RCLCPP_INFO(this->get_logger(), "Action request to ROSA completed: %s", action_name_.c_str());
      return ActionExecutorClient::on_activate(previous_state);
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to start action %s", action_name_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  RosaAction::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Action cancelation requested to ROSA: %s", action_name_.c_str());

    while (!action_req_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(this->get_logger(), "service /rosa_kb/action/request not available, waiting again...");
    }

    auto request = std::make_shared<rosa_msgs::srv::ActionQuery::Request>();
    request->action.name = action_name_;
    request->action.is_required = false;

    auto response = action_req_client_->async_send_request(request);
    if (response.wait_for(1s) == std::future_status::ready && response.get()->success == true)
    {
      RCLCPP_INFO(this->get_logger(), "Action cancelation request to ROSA completed: %s", action_name_.c_str());
      return ActionExecutorClient::on_deactivate(previous_state);
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to stop action %s", action_name_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

}  // namespace rosa_task_plan_plansys
