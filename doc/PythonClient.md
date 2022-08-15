# Simple Action Client (Python)

## Minimal Example
Sends the goal, and ignores the result and the feedback
```python
import rclpy
from rclpy.node import Node
from simple_actions import SimpleActionClient
from action_tutorials_interfaces.action import Fibonacci


def main(args=None):
    rclpy.init(args=args)
    node = Node('my_client')
    client = SimpleActionClient(node, Fibonacci, 'fibonacci')

    goal_msg = Fibonacci.Goal(order=10)
    client.send_goal(goal_msg)


if __name__ == '__main__':
    main()
```

## Example with Result
This example sends the goal and waits for the result.

```python
import rclpy
from rclpy.node import Node
from simple_actions import SimpleActionClient
from action_tutorials_interfaces.action import Fibonacci


def main(args=None):
    rclpy.init(args=args)
    node = Node('my_client')
    client = SimpleActionClient(node, Fibonacci, 'fibonacci')

    goal_msg = Fibonacci.Goal(order=10)
    # Use the call operator to send goal
    result = client(goal_msg)
    print(result.sequence)


if __name__ == '__main__':
    main()
```

## Example with Callbacks
This example uses custom callbacks for the result and feedback.
```python
import rclpy
from rclpy.node import Node
from simple_actions import SimpleActionClient
from action_tutorials_interfaces.action import Fibonacci


def my_awesome_feedback_cb(feedback):
    print(f'Feedback: {feedback.partial_sequence}')


def my_awesome_result_cb(result_code, result):
    print(f'Result ({result_code}): {result.sequence}')


def main(args=None):
    rclpy.init(args=args)
    node = Node('my_client')
    client = SimpleActionClient(node, Fibonacci, 'fibonacci')

    goal_msg = Fibonacci.Goal(order=10)
    client.send_goal(goal_msg,
                     result_callback=my_awesome_result_cb,
                     feedback_callback=my_awesome_feedback_cb)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

### Call Operator with Feedback
You can also specify a feedback callback with the call operator, a la
```python
    result = client(goal_msg, feedback_callback=my_awesome_feedback_cb)
```

## Wait For Server
By default, creating the `SimpleActionClient` will wait for the server to come up. You can disable that by passing in `wait_for_server=False` and then calling `wait_for_server()` later.

```python
    client = SimpleActionClient(node, Fibonacci, 'fibonacci', wait_for_server=False)
    # Do something else
    client.wait_for_server()
    client.send_goal(goal_msg)
```
