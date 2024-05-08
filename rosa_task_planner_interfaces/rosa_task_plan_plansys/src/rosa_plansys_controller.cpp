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
#include "rosa_task_plan_plansys/rosa_plansys_controller.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rosa_task_plan_plansys
{

  RosaPlansysController::RosaPlansysController(const std::string & node_name)
  : rclcpp::Node(node_name)
{
  this->declare_parameter("rosa_actions", rclcpp::PARAMETER_STRING_ARRAY);

  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>("rosa_plansys_controller_executor");

  callback_group_actions_client_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  selectable_actions_client_ =
    this->create_client<rosa_msgs::srv::ActionQueryArray>(
      "/rosa_kb/action/selectable",
      rmw_qos_profile_services_default,
      callback_group_actions_client_);

  rosa_events_sub_ = create_subscription<std_msgs::msg::String>(
    "/rosa_kb/events",
    10,
    std::bind(&RosaPlansysController::rosa_events_cb, this, _1));

  step_timer_cb_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  // TODO: create parameter for timer rate?
  step_timer_ = this->create_wall_timer(
    1s, std::bind(&RosaPlansysController::step, this), step_timer_cb_group_);
}

RosaPlansysController::~RosaPlansysController()
{
}

void RosaPlansysController::rosa_events_cb(const std_msgs::msg::String msg){
  if (msg.data == "insert_monitoring_data" || msg.data == "action_update"){
    this->update_actions_feasibility();
  }
}

bool RosaPlansysController::update_actions_feasibility(){
  auto rosa_actions = this->get_parameter("rosa_actions").as_string_array();

  while (!selectable_actions_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "/rosa_kb/action/selectable service not available, waiting again...");
  }

  auto request = std::make_shared<rosa_msgs::srv::ActionQueryArray::Request>();
  auto response = selectable_actions_client_->async_send_request(request);
  if (response.wait_for(1s) == std::future_status::ready)
  {
    auto srv_response = response.get();
    if (srv_response->success == false) return false;

    auto selectable_actions = srv_response->actions;
    for (auto action: rosa_actions){
      if(std::any_of(selectable_actions.begin(), selectable_actions.end(),
        [&](const rosa_msgs::msg::Action& elem) { return elem.name.c_str() == action; })) {
        problem_expert_->addPredicate(plansys2::Predicate("(action_feasible "+action+")"));
      } else{
        problem_expert_->removePredicate(plansys2::Predicate("(action_feasible "+action+")"));
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service actions selectable");
    return false;
  }
  return true;
}

void RosaPlansysController::execute_plan(){
  // Compute the plan
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    for (auto instance: problem_expert_->getInstances()){
      std::cout<<"Instance "<< instance.name.c_str() << " type " <<
        instance.type.c_str() << std::endl;
    }
    for (auto predicate: problem_expert_->getPredicates()) {
      std::cout << "Predicates: " << std::endl;
      std::cout << parser::pddl::toString(predicate)<<std::endl;
    }

    std::cout << "Could not find plan to reach goal " <<
     parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    return;
  }

  std::cout << "Selected plan: " << std::endl;
  for (auto item : plan->items){
    RCLCPP_INFO(this->get_logger(), "  Action: '%s'", item.action.c_str());
  }
  // Execute the plan
  executor_client_->start_plan_execution(plan.value());
}

void RosaPlansysController::step(){
  if (first_iteration_ && this->update_actions_feasibility()){
    this->execute_plan();
    first_iteration_ = false;
    return;
  }

  if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
    if (executor_client_->getResult().value().success) {
      std::cout << "Successful finished " << std::endl;
      mission_completed = true;
    } else {
        std::cout << "Replanning!" << std::endl;
        this->execute_plan();
        return;
    }
  }

  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::string error_str_ = "[" + action_feedback.action + "] finished with error: " + action_feedback.message_status;
      RCLCPP_ERROR(this->get_logger(), error_str_.c_str());
      break;
    }

    std::string arguments_str_ = " ";
    for (const auto & arguments: action_feedback.arguments){
      arguments_str_ += arguments + " ";
    }
    std::string feedback_str_ = "[" + action_feedback.action + arguments_str_ +
      std::to_string(action_feedback.completion * 100.0) + "%]";
    RCLCPP_INFO(this->get_logger(), feedback_str_.c_str());
    // std::cout << "[" << action_feedback.action << arguments_str <<
      // action_feedback.completion * 100.0 << "%]";
    // std::cout << std::endl;
  }

  if(mission_completed) step_timer_->cancel();
}


}  // namespace rosa_task_plan_plansys
