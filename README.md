# simple_actions

A simple version of the `rclpy/rclcpp` action libraries.

## Key Design Principles
 * For use with *simple* situations where there will never be two active goals.
 * Made for people like me who can't think about the `future` for too long without their head hurting.
 * Callbacks/methods should deal with the regular `Goal/Feedback/Result` methods, not wrapped versions.
 * If the Action Client does not connect to the server relatively quickly, a warning should be printed while it waits, and another message should report when it finally does connect.

## Implementations

 * [SimpleActionServer (Python)](doc/PythonServer.md)
 * [SimpleActionServer (C++)](doc/CPlusPlusServer.md)
 * [SimpleActionClient (Python)](doc/PythonClient.md)
 * [SimpleActionClient (C++)](doc/CPlusPlusClient.md)
