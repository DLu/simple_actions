import rclpy
from rclpy.action import ActionClient


class SimpleActionClient:
    """Library for easier use of rclpy action client

    See README.md for usage."""

    def __init__(self, node, action_type, action_name, wait_for_server=True):
        self.node = node

        class_type = str(action_type).split("'")[1]  # Hacky way to clean up the type string
        self.info_string = f'{action_name}/{class_type}'

        self.action_client = ActionClient(node, action_type, action_name)
        if wait_for_server:
            self.wait_for_server()
        self.result = None

    def wait_for_server(self):
        printed = False
        while not self.action_client.wait_for_server(timeout_sec=10.0):
            if not printed:
                self.node.get_logger().warn(f'Waiting for {self.info_string}')
                printed = True
        if printed:
            self.node.get_logger().info(f'Connected to {self.info_string}')

    def send_goal(self, goal_msg, result_callback=None, feedback_callback=None):
        self.result_callback = result_callback
        self.feedback_callback = feedback_callback
        self.result = None

        actual_callback = self._feedback_callback if self.feedback_callback else None
        self._goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=actual_callback)
        self._goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.result_callback:
                self.result_callback(None)
            self.get_logger().error('Goal rejected :(')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.result = future.result().result
        if self.result_callback:
            self.result_callback(self.result)

    def _feedback_callback(self, feedback_msg):
        if self.feedback_callback:
            self.feedback_callback(feedback_msg.feedback)

    def __call__(self, goal_msg, feedback_callback=None):
        self.send_goal(goal_msg, feedback_callback=feedback_callback)
        while self.result is None:
            rclpy.spin_once(self.node)
        return self.result

    def __repr__(self):
        return 'SimpleActionClient({})'.format(self.info_string)
