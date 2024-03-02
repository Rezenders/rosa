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
#include "rosa_task_plan_plansys/rosa_action_plansys.hpp"

using namespace std::chrono_literals;
namespace rosa_task_plan_plansys
{

  RosaActionPlansys::RosaActionPlansys(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  :   plansys2::ActionExecutorClient(node_name, rate), action_name_("rosa_action")
  {
    action_req_client_ =
      this->create_client<rosa_msgs::srv::ActionQuery>("/rosa_kb/action/request");
  }

  RosaActionPlansys::~RosaActionPlansys()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  RosaActionPlansys::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Action requested: %s", action_name_.c_str());

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
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true)
    {
      RCLCPP_INFO(this->get_logger(), "Action request completed: %s", action_name_.c_str());
      return ActionExecutorClient::on_activate(previous_state);
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to start action %s", action_name_.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  RosaActionPlansys::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Action cancelation requested: %s", action_name_.c_str());

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
    if (!(rclcpp::spin_until_future_complete(this->get_node_base_interface(), response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to stop action %s", action_name_.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "Action cancelation completed: %s", action_name_.c_str());
    return ActionExecutorClient::on_deactivate(previous_state);
  }

}  // namespace rosa_task_plan_plansys
