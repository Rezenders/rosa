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

#include "metacontrol_plan/metacontroled_action.hpp"

namespace metacontrol_plan
{

  using namespace std::chrono_literals;

  MetacontroledAction::MetacontroledAction(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    task_req_client =
      node_->create_client<metacontrol_kb_msgs::srv::TaskRequest>("/metacontrol_kb/task/request");
  }

  BT::NodeStatus MetacontroledAction::onStart(){
    std::cout << "Async action starting: " << this->name() << std::endl;

    while (!task_req_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }

    auto request = std::make_shared<metacontrol_kb_msgs::srv::TaskRequest::Request>();
    request->task.task_name = this->name();
    request->required = true;

    auto response = task_req_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true)
    {
      return BT::NodeStatus::RUNNING;
    } else{
      std::cout << "action failed: " << this->name() << std::endl;
      RCLCPP_ERROR(node_->get_logger(), "Failed to start action %s", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  void MetacontroledAction::onHalted(){
    std::cout<< "Async action halted: "<< this->name() <<std::endl;

    while (!task_req_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }

    auto request = std::make_shared<metacontrol_kb_msgs::srv::TaskRequest::Request>();
    request->task.task_name = this->name();
    request->required = false;

    auto response = task_req_client->async_send_request(request);
    if (!(rclcpp::spin_until_future_complete(node_, response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to stop action %s", this->name().c_str());
    }
  }
} //namespace metacontrol_plan
