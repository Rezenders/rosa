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

#include "rosa_plan/rosa_action.hpp"

namespace rosa_plan
{

  using namespace std::chrono_literals;

  RosaAction::RosaAction(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    task_req_client =
      node_->create_client<rosa_msgs::srv::TaskRequest>("/rosa_kb/task/request");
  }

  BT::NodeStatus RosaAction::onStart(){
    std::cout << "Async action starting: " << this->name() << std::endl;
    RCLCPP_INFO(node_->get_logger(), "Task requested: %s", this->name().c_str());

    while (!task_req_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "service /rosa_kb/task/request not available, waiting again...");
    }

    auto request = std::make_shared<rosa_msgs::srv::TaskRequest::Request>();
    request->task.task_name = this->name();
    request->required = true;

    auto response = task_req_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true)
    {
      RCLCPP_INFO(node_->get_logger(), "Task request completed: %s", this->name().c_str());
      return BT::NodeStatus::RUNNING;
    } else{
      std::cout << "action failed: " << this->name() << std::endl;
      RCLCPP_ERROR(node_->get_logger(), "Failed to start action %s", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  void RosaAction::cancel_task(){
    RCLCPP_INFO(node_->get_logger(), "Task cancelation requested: %s", this->name().c_str());

    while (!task_req_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(node_->get_logger(), "service /rosa_kb/task/request not available, waiting again...");
    }

    auto request = std::make_shared<rosa_msgs::srv::TaskRequest::Request>();
    request->task.task_name = this->name();
    request->required = false;

    auto response = task_req_client->async_send_request(request);
    if (!(rclcpp::spin_until_future_complete(node_, response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to stop action %s", this->name().c_str());
    } else{
      RCLCPP_INFO(node_->get_logger(), "Task cancelation completed: %s", this->name().c_str());
    }
  }

  void RosaAction::onHalted(){
    RCLCPP_INFO(node_->get_logger(), "Async action halted: %s", this->name().c_str());
    cancel_task();
  }
} //namespace rosa_plan
