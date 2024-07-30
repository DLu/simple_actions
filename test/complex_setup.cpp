/*********************************************************************
 * Software License Agreement (BSD License 2.0)
 *
 *  Copyright (c) 2023, Metro Robots
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

#include <gtest/gtest.h>
#include <simple_actions/simple_client.hpp>
#include <simple_actions/simple_server.hpp>
#include <action_tutorials_interfaces/action/fibonacci.hpp>

#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_client.hpp>

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class TestComplexSetup : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    // Instantiate the action client we test with
    server_node_ = std::make_shared<rclcpp::Node>("server_node");
    fibonacci_action_server_ = std::make_shared<simple_actions::SimpleActionServer<Fibonacci>>(
        server_node_, "/fibonacci", [](const Fibonacci::Goal& /*goal*/, Fibonacci::Result& result) {
          result.sequence = {0, 1, 1, 2, 3, 5, 8, 13};
          return true;
        });

    client_node_ = std::make_shared<rclcpp::Node>("client_node");
    fibonacci_action_client_ =
        std::make_shared<simple_actions::SimpleActionClient<Fibonacci>>(client_node_, "/fibonacci");

    executor_.add_node(server_node_);
    executor_.add_node(client_node_);

    executor_future_handle_ = std::async(std::launch::async, [&]() -> void { executor_.spin(); });
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void TearDown()
  {
    // Clean up resources
    fibonacci_action_server_.reset();
    fibonacci_action_client_.reset();

    executor_.cancel();
  }

  std::shared_ptr<rclcpp::Node> server_node_;
  std::shared_ptr<rclcpp::Node> client_node_;

  std::shared_ptr<simple_actions::SimpleActionServer<Fibonacci>> fibonacci_action_server_;
  std::shared_ptr<simple_actions::SimpleActionClient<Fibonacci>> fibonacci_action_client_;

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::future<void> executor_future_handle_;
};

TEST_F(TestComplexSetup, succeeds_fine)
{
  Fibonacci::Goal goal_msg;
  goal_msg.order = 10;
  auto result = fibonacci_action_client_->execute(goal_msg, nullptr, false);
  EXPECT_EQ(fibonacci_action_client_->getLatestResultCode(), simple_actions::ResultCode::SUCCEEDED);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
