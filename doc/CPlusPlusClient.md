# Simple Action Client (C++)

## Minimal Example
Sends the goal, and ignores the result and the feedback
```cpp
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_client.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client_demo");
  simple_actions::SimpleActionClient<action_tutorials_interfaces::action::Fibonacci> client(node, "fibonacci");

  action_tutorials_interfaces::action::Fibonacci::Goal goal_msg;
  goal_msg.order = 10;
  client.sendGoal(goal_msg);
  rclcpp::shutdown();
  return 0;
}
```

## Example with Result
This example sends the goal and waits for the result.

```cpp
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_client.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client_demo");
  simple_actions::SimpleActionClient<action_tutorials_interfaces::action::Fibonacci> client(node, "fibonacci");

  action_tutorials_interfaces::action::Fibonacci::Goal goal_msg;
  goal_msg.order = 10;
  auto result = client.execute(goal_msg);
  for (const auto& n : result.sequence)
  {
    RCLCPP_INFO(node->get_logger(), "%d", n);
  }
  rclcpp::shutdown();
  return 0;
}
```

## Example with Callbacks
This example uses custom callbacks for the result and feedback.
```cpp
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_client.hpp>

void feedbackCallback(const action_tutorials_interfaces::action::Fibonacci::Feedback& feedback)
{
  std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (auto number : feedback.partial_sequence) {
    ss << number << " ";
  }
  std::cout << ss.str() << std::endl;
}

void resultCallback(simple_actions::ResultCode code,
                    const action_tutorials_interfaces::action::Fibonacci::Result& result)
{
  if (code == simple_actions::ResultCode::SUCCEEDED)
  {
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.sequence) {
      ss << number << " ";
    }
    std::cout << ss.str().c_str() << std::endl;
  }
  else
  {
    std::cerr << "Action unsuccessful." << std::endl;
  }
  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  using namespace std::placeholders;
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client_demo");
  simple_actions::SimpleActionClient<action_tutorials_interfaces::action::Fibonacci> client(node, "fibonacci");

  action_tutorials_interfaces::action::Fibonacci::Goal goal_msg;
  goal_msg.order = 10;
  client.sendGoal(goal_msg, std::bind(resultCallback, _1, _2), std::bind(feedbackCallback, _1));
  rclcpp::spin(node);
  return 0;
}
```

### Example With Class Callbacks
```cpp
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <simple_actions/simple_client.hpp>

class FibonacciActionClient
{
public:
  explicit FibonacciActionClient(rclcpp::Node::SharedPtr node)
  : node_(node), client_(node_, "fibonacci")
  {
  }

  void sendGoal()
  {
    using namespace std::placeholders;

    auto goal_msg = action_tutorials_interfaces::action::Fibonacci::Goal();
    goal_msg.order = 10;

    client_.sendGoal(goal_msg, std::bind(&FibonacciActionClient::resultCallback, this, _1, _2), std::bind(&FibonacciActionClient::feedbackCallback, this, _1));
  }

private:
  rclcpp::Node::SharedPtr node_;
  simple_actions::SimpleActionClient<action_tutorials_interfaces::action::Fibonacci> client_;

  void feedbackCallback(const action_tutorials_interfaces::action::Fibonacci::Feedback& feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback.partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
  }

  void resultCallback(simple_actions::ResultCode code,
                       const action_tutorials_interfaces::action::Fibonacci::Result & result)
  {
    if (code == simple_actions::ResultCode::SUCCEEDED)
    {
      std::stringstream ss;
      ss << "Result received: ";
      for (auto number : result.sequence)
      {
        ss << number << " ";
      }
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Action unsuccessful.");
    }
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("client_demo");
  FibonacciActionClient ac(node);
  ac.sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Execute with Feedback
You can also specify a feedback callback with the call operator, a la
```cpp
    auto result = client.execute(goal_msg, std::bind(feedbackCallback, _1));
```

## Wait For Server
By default, creating the `SimpleActionClient` will wait for the server to come up. You can disable that by passing in `wait_for_server=False` and then calling `waitForServer()` later.

```cpp
    simple_actions::SimpleActionClient<action_tutorials_interfaces::action::Fibonacci> client(node, "fibonacci", false);

    # Do something else
    client.waitForServer();
    client.sendGoal(goal_msg);
```
