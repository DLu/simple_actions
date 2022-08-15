# Software License Agreement (BSD License 2.0)
#
# Copyright (c) 2022, Metro Robots
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Metro Robots nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import rclpy
from rclpy.action import ActionClient
from enum import Enum
from .utilities import get_action_name


class ResultCode(Enum):
    """Similar to action_msgs/msg/GoalStatus but also contains REJECTED."""

    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    ABORTED = 3
    REJECTED = 4


class SimpleActionClient:
    """
    Simple wrapper around rclpy action client for easier usage. Assumes only one action at a time.

    The ResultCallback should take two parameters, a ResultCode and the Action result

    The FeedbackCallback should take a single parameter for the Action feedback
    """

    def __init__(self, node, action_type, action_namespace, wait_for_server=True):
        self.node = node
        self.logger = self.node.get_logger()

        self.info_string = f'{action_namespace}/{get_action_name(action_type)}'

        self.action_client = ActionClient(node, action_type, action_namespace)
        if wait_for_server:
            self.wait_for_server()
        self._result = None
        self.logger.debug(f'{self} initialized')

    def wait_for_server(self):
        """Wait indefinitely for the server to come up. Will print a message after 10 seconds."""
        printed = False
        while not self.action_client.wait_for_server(timeout_sec=10.0):
            if not printed:
                self.logger.warn(f'Waiting for {self.info_string}')
                printed = True
        if printed:
            self.logger.info(f'Connected to {self.info_string}')
        else:
            self.logger.debug(f'{self} connected')

    def send_goal(self, goal_msg, result_callback=None, feedback_callback=None):
        """Send a goal, and return immediately."""
        self.result_callback = result_callback
        self.feedback_callback = feedback_callback
        self._result = None

        actual_callback = self._feedback_callback if self.feedback_callback else None
        self._goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=actual_callback)
        self.logger.debug(f'{self} sent goal')
        self._goal_future.add_done_callback(self._goal_response_callback)

    def __call__(self, goal_msg, feedback_callback=None):
        """Send a goal, and wait for the result."""
        self.send_goal(goal_msg, feedback_callback=feedback_callback)
        while self._result is None and rclpy.ok():
            rclpy.spin_once(self.node)
        return self._result

    def _goal_response_callback(self, future):
        # Internal callback
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.result_callback:
                self.result_callback(ResultCode.REJECTED, None)
            self.logger.debug('{self} goal rejected :(')
            return
        self.logger.debug(f'{self} goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        # Internal callback
        self.logger.debug(f'{self} got feedback')
        if self.feedback_callback:
            self.feedback_callback(feedback_msg.feedback)

    def _result_callback(self, future):
        # Internal callback
        self.logger.debug(f'{self} got result')
        super_result = future.result()

        # Translate action_msgs/msg/GoalStatus to ResultCode
        result_code = [
            ResultCode.UNKNOWN,    # STATUS_UNKNOWN
            ResultCode.UNKNOWN,    # STATUS_ACCEPTED
            ResultCode.UNKNOWN,    # STATUS_EXECUTING
            ResultCode.UNKNOWN,    # STATUS_CANCELING
            ResultCode.SUCCEEDED,  # STATUS_SUCCEEDED
            ResultCode.CANCELED,   # STATUS_CANCELED
            ResultCode.ABORTED     # STATUS_ABORTED
        ][super_result.status]

        self._result = result_code, super_result.result
        if self.result_callback:
            self.result_callback(*self._result)

    def __repr__(self):
        return f'SimpleActionClient({self.info_string})'
