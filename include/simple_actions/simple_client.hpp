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
 * @brief Similar to rclcpp_action::ResultCode but also contains REJECTED
 */
enum ResultCode : int8_t { UNKNOWN, SUCCEEDED, CANCELED, ABORTED, REJECTED };

/**
 * @brief Simple wrapper around rclcpp action client for easier usage. Assumes only one action at a time.
 */
template <typename ACTION_TYPE>
class SimpleActionClient
{
public:
  using FeedbackCallback = std::function<void(const typename ACTION_TYPE::Feedback&)>;
  using ResultCallback = std::function<void(const ResultCode&, const typename ACTION_TYPE::Result&)>;

  SimpleActionClient(rclcpp::Node::SharedPtr node, const std::string& action_namespace, bool wait_for_server = true)
    : node_(node), LOGGER(node_->get_logger().get_child(action_namespace)), action_namespace_(action_namespace)
  {
    client_ = rclcpp_action::create_client<ACTION_TYPE>(node_, action_namespace);

    info_string_ = action_namespace + "/" + getName<ACTION_TYPE>();
    if (wait_for_server)
    {
      waitForServer();
    }
    RCLCPP_DEBUG(LOGGER, "%s initialized", info_string_.c_str());
  }

  /**
   * @brief Waits indefinitely for the server to come up. Will print a message after 10 seconds.
   */
  void waitForServer()
  {
    bool printed = false;
    using namespace std::chrono_literals;
    while (!client_->wait_for_action_server(10s) && rclcpp::ok())
    {
      if (!printed)
      {
        RCLCPP_WARN(LOGGER, "Waiting for %s", info_string_.c_str());
        printed = true;
      }
    }
    if (!rclcpp::ok())
    {
      exit(-1);
    }
    if (printed)
    {
      RCLCPP_INFO(LOGGER, "Connected to %s", info_string_.c_str());
    }
    else
    {
      RCLCPP_DEBUG(LOGGER, "%s connected", info_string_.c_str());
    }
  }

  /**
   * @brief Send a goal, and return immediately
   */
  void sendGoal(const typename ACTION_TYPE::Goal& goal_msg, ResultCallback resultCB = nullptr,
                FeedbackCallback feedbackCB = nullptr)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    typename rclcpp_action::Client<ACTION_TYPE>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goalResponseCallback, this, _1);

    if (feedbackCB)
    {
      feedback_cb_ = feedbackCB;
      send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);
    }

    result_cb_ = resultCB;
    send_goal_options.result_callback = std::bind(&SimpleActionClient::resultCallback, this, _1);
    client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_DEBUG(LOGGER, "%s sent goal", info_string_.c_str());
  }

  /**
   * @brief Send a goal and wait for the result.
   *
   * @note: Does not return the ResultCode
   */
  typename ACTION_TYPE::Result& execute(const typename ACTION_TYPE::Goal& goal_msg,
                                        FeedbackCallback feedbackCB = nullptr)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    execute_result_recieved_ = false;
    sendGoal(goal_msg, std::bind(&SimpleActionClient::executeResultCallback, this, _1, _2), feedbackCB);
    while (!execute_result_recieved_ && rclcpp::ok())
    {
      rclcpp::spin_some(node_);
    }
    return execute_result_;
  }

protected:
  void goalResponseCallback(std::shared_future<typename rclcpp_action::ClientGoalHandle<ACTION_TYPE>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle)
    {
      if (result_cb_)
      {
        result_cb_(ResultCode::REJECTED, default_result_);
      }
      RCLCPP_DEBUG(LOGGER, "%s goal rejected :(", info_string_.c_str());
      return;
    }
    RCLCPP_DEBUG(LOGGER, "%s goal accepted :)", info_string_.c_str());
  }

  void feedbackCallback(typename rclcpp_action::ClientGoalHandle<ACTION_TYPE>::SharedPtr,
                        const std::shared_ptr<const typename ACTION_TYPE::Feedback> feedback)
  {
    RCLCPP_DEBUG(LOGGER, "%s got feedback", info_string_.c_str());
    if (feedback_cb_)
    {
      feedback_cb_(*feedback);
    }
  }

  void resultCallback(const typename rclcpp_action::ClientGoalHandle<ACTION_TYPE>::WrappedResult& result)
  {
    RCLCPP_DEBUG(LOGGER, "%s got result", info_string_.c_str());
    if (result_cb_)
    {
      ResultCode code;
      // Translate from rclcpp_action::ResultCode to simple_actions::ResultCode
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        code = ResultCode::SUCCEEDED;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        code = ResultCode::ABORTED;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        code = ResultCode::CANCELED;
        break;
      default:
        code = ResultCode::UNKNOWN;
      }
      result_cb_(code, *(result.result));
    }
  }

  void executeResultCallback(ResultCode, const typename ACTION_TYPE::Result& result)
  {
    execute_result_ = result;
    execute_result_recieved_ = true;
  }

  typename rclcpp_action::Client<ACTION_TYPE>::SharedPtr client_;
  typename ACTION_TYPE::Result default_result_, execute_result_;
  bool execute_result_recieved_;

  FeedbackCallback feedback_cb_;
  ResultCallback result_cb_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger LOGGER;
  const std::string& action_namespace_;
  std::string info_string_;
};
}  // namespace simple_actions
