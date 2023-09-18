/*********************************************************************
 * Software License Agreement (BSD License 2.0)
 *
 *  Copyright (c) 2022, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <simple_actions/utilities.hpp>

namespace simple_actions
{
/**
 * @brief Simple wrapper around rclcpp action server for easier usage. Assumes only one action at a time.
 *
 * Execute callback should return true if successful and write into the result object
 */
template <typename ACTION_TYPE>
class SimpleActionServer
{
public:
  using ExecuteCallback = std::function<bool(const typename ACTION_TYPE::Goal&, typename ACTION_TYPE::Result&)>;

  SimpleActionServer(rclcpp::Node::SharedPtr node, const std::string& action_namespace, ExecuteCallback execute_cb)
    : node_(node), LOGGER(node_->get_logger().get_child(action_namespace))
  {
    using namespace std::placeholders;
    info_string_ = action_namespace + "/" + getName<ACTION_TYPE>();
    execute_cb_ = execute_cb;
    server_ = rclcpp_action::create_server<ACTION_TYPE>(node_, action_namespace,
                                                        std::bind(&SimpleActionServer::handleGoal, this, _1, _2),
                                                        std::bind(&SimpleActionServer::handleCancel, this, _1),
                                                        std::bind(&SimpleActionServer::handleAccepted, this, _1));
    RCLCPP_DEBUG(LOGGER, "%s initialized", info_string_.c_str());
  }

  /**
   * @brief Publishes feedback on the active action
   */
  void publishFeedback(typename ACTION_TYPE::Feedback::SharedPtr feedback)
  {
    goal_handle_->publish_feedback(feedback);
    RCLCPP_DEBUG(LOGGER, "%s published feedback", info_string_.c_str());
  }

  bool isActive() const
  {
    return goal_handle_->is_active();
  }

  bool isCanceling() const
  {
    return goal_handle_->is_canceling();
  }

  bool isExecuting() const
  {
    return goal_handle_->is_executing();
  }

protected:
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<ACTION_TYPE>>;

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                         std::shared_ptr<const typename ACTION_TYPE::Goal>)
  {
    RCLCPP_DEBUG(LOGGER, "%s got a goal request", info_string_.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const GoalHandle)
  {
    RCLCPP_DEBUG(LOGGER, "%s got a cancel request", info_string_.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const GoalHandle goal_handle)
  {
    RCLCPP_DEBUG(LOGGER, "%s is starting an execution thread", info_string_.c_str());
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SimpleActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(GoalHandle goal_handle)
  {
    RCLCPP_DEBUG(LOGGER, "%s is beginning execution", info_string_.c_str());
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<typename ACTION_TYPE::Result>();

    bool success;
    try
    {
      success = execute_cb_(*goal, *result);
    }
    catch (const std::exception& ex)
    {
      success = false;
      RCLCPP_ERROR(LOGGER, "An uncaught exception has occurred while trying to execute the action %s: %s",
                   info_string_.c_str(), ex.what());
    }

    if (success)
    {
      goal_handle->succeed(result);
    }
    else if (goal_handle_->is_canceling())
    {
      goal_handle->canceled(result);
    }
    else
    {
      goal_handle->abort(result);
    }
    RCLCPP_DEBUG(LOGGER, "%s has finished execution", info_string_.c_str());
  }

  typename rclcpp_action::Server<ACTION_TYPE>::SharedPtr server_;

  GoalHandle goal_handle_;
  ExecuteCallback execute_cb_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger LOGGER;
  std::string info_string_;
};
}  // namespace simple_actions
