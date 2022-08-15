# Simple Action Server (C++)

## Minimal Example
Listens for goal, returns the result (no feedback)

```cpp
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_server.hpp>

bool execute(const action_tutorials_interfaces::action::Fibonacci::Goal& goal,
             action_tutorials_interfaces::action::Fibonacci::Result& result)
{
  rclcpp::Rate loop_rate(1);
  std::vector<int> sequence;
  sequence.push_back(0);
  sequence.push_back(1);

  for (int i = 1; i < goal.order; ++i)
  {
    sequence.push_back(sequence[i] + sequence[i - 1]);
    loop_rate.sleep();
  }

  result.sequence = sequence;
  return true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("server_demo");
  simple_actions::SimpleActionServer<action_tutorials_interfaces::action::Fibonacci> server(
      node, "fibonacci", std::bind(&execute, std::placeholders::_1, std::placeholders::_2));
  rclcpp::spin(node);
  return 0;
}
```

## Class Version with Feedback
You need the `action_server` reference to send feedback, so its easier to do it within a class then deal with global variables or lambdas. YMMV.

```cpp
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_server.hpp>

class FibonacciActionServer
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

  explicit FibonacciActionServer(rclcpp::Node::SharedPtr node)
    : node_(node),
      server_(node_, "fibonacci",
              std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1, std::placeholders::_2))
  {
  }

private:
  rclcpp::Node::SharedPtr node_;
  simple_actions::SimpleActionServer<Fibonacci> server_;

  bool execute(const Fibonacci::Goal& goal, Fibonacci::Result& result)
  {
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto& sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    for (int i = 1; (i < goal.order) && rclcpp::ok(); ++i)
    {
      sequence.push_back(sequence[i] + sequence[i - 1]);
      server_.publishFeedback(feedback);
      loop_rate.sleep();
    }

    result.sequence = sequence;
    return true;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("server_demo");
  FibonacciActionServer as(node);
  rclcpp::spin(node);
  return 0;
}

```
