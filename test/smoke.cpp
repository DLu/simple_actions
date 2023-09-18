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

bool execute(const action_tutorials_interfaces::action::Fibonacci::Goal&,
             action_tutorials_interfaces::action::Fibonacci::Result&)
{
  return false;
}

TEST(Smoke, clientTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client_demo");
  simple_actions::SimpleActionClient<action_tutorials_interfaces::action::Fibonacci> client(node, "fibonacci", false);
}

TEST(Smoke, serverTest)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("server_demo");
  simple_actions::SimpleActionServer<action_tutorials_interfaces::action::Fibonacci> server(
      node, "fibonacci", std::bind(&execute, std::placeholders::_1, std::placeholders::_2));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
