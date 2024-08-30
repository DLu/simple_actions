# Simple Action Server (Python)

## Minimal Example
Listens for goal, returns the result (no feedback)

```python
import time
import rclpy
from rclpy.node import Node
from simple_actions import SimpleActionServer
from example_interfaces.action import Fibonacci


def fibonacci_action(goal):
    seq = [0, 1]

    for i in range(1, goal.order):
        seq.append(seq[i] + seq[i - 1])
        time.sleep(1.0)

    result = Fibonacci.Result()
    result.sequence = seq
    return result


def main(args=None):
    rclpy.init(args=args)
    node = Node('my_server')
    SimpleActionServer(node, Fibonacci, 'fibonacci', fibonacci_action)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

## Class Version with Feedback
You need the `action_server` handle to send feedback, so its easier to do it within a class then deal with global variables or lambdas. YMMV.

```python
#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from simple_actions import SimpleActionServer
from example_interfaces.action import Fibonacci


class FibonacciServer:
    def __init__(self, node):
        self.action_server = SimpleActionServer(node, Fibonacci, 'fibonacci', self.fibonacci_action)

    def fibonacci_action(self, goal):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            self.action_server.publish_feedback(feedback_msg)
            time.sleep(1.0)

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Node('my_server')
    FibonacciServer(node)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```
