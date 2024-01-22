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

#ifndef ROSA_PLAN__ROSA_ACTION_HPP_
#define ROSA_PLAN__ROSA_ACTION_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rosa_msgs/srv/action_query.hpp"

using namespace std::chrono_literals;

namespace rosa_plan
{

template<class T>
class RosaAction : public BT::StatefulActionNode{

public:
  RosaAction(
    const std::string& name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    _node = config().blackboard->get<T>("node");
    action_req_client =
      _node->template create_client<rosa_msgs::srv::ActionQuery>("/rosa_kb/action/request");
  };

  BT::NodeStatus onStart() override {
    std::cout << "Async action starting: " << this->name() << std::endl;
    RCLCPP_INFO(_node->get_logger(), "Action requested: %s", this->name().c_str());

    while (!action_req_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(_node->get_logger(), "service /rosa_kb/action/request not available, waiting again...");
    }

    auto request = std::make_shared<rosa_msgs::srv::ActionQuery::Request>();
    request->action.name = this->name();
    request->action.is_required = true;

    auto response = action_req_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(_node, response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true)
    {
      RCLCPP_INFO(_node->get_logger(), "Action request completed: %s", this->name().c_str());
      return BT::NodeStatus::RUNNING;
    } else{
      std::cout << "action failed: " << this->name() << std::endl;
      RCLCPP_ERROR(_node->get_logger(), "Failed to start action %s", this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  };

  void onHalted() override{
    RCLCPP_INFO(_node->get_logger(), "Async action halted: %s", this->name().c_str());
    cancel_action();
  };

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
      });
  }

protected:
  T _node;
  rclcpp::Client<rosa_msgs::srv::ActionQuery>::SharedPtr action_req_client;

  void cancel_action(){
    RCLCPP_INFO(_node->get_logger(), "Action cancelation requested: %s", this->name().c_str());

    while (!action_req_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(_node->get_logger(), "service /rosa_kb/action/request not available, waiting again...");
    }

    auto request = std::make_shared<rosa_msgs::srv::ActionQuery::Request>();
    request->action.name = this->name();
    request->action.is_required = false;

    auto response = action_req_client->async_send_request(request);
    if (!(rclcpp::spin_until_future_complete(_node, response) ==
      rclcpp::FutureReturnCode::SUCCESS && response.get()->success == true))
    {
      RCLCPP_ERROR(_node->get_logger(), "Failed to stop action %s", this->name().c_str());
    } else{
      RCLCPP_INFO(_node->get_logger(), "Action cancelation completed: %s", this->name().c_str());
    }
  };
};

}  // namespace rosa_plan

#endif  // ROSA_PLAN__ROSA_ACTION_HPP_
