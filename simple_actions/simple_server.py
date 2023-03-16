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

from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus
from .utilities import get_action_name


class SimpleActionServer:
    """
    Simple wrapper around rclpy action server for "easier" usage. Assumes only one action at a time.

    The Execute callback should return a Result if successful. Otherwise it will be unsuccessful.
    """

    def __init__(self, node, action_type, action_namespace, execute_callback):
        self.logger = node.get_logger()
        self.action_type = action_type
        self.execute_callback = execute_callback
        self.info_string = f'{action_namespace}/{get_action_name(action_type)}'
        self.action_server = ActionServer(node, action_type, action_namespace, self._execute_callback)
        self.goal_handle = None
        self.logger.debug(f'{self} initialized')

    def publish_feedback(self, feedback):
        """Publish feedback on the active action."""
        self.goal_handle.publish_feedback(feedback)
        self.logger.debug(f'{self} published feedback')

    def is_active(self):
        return self.goal_handle and self.goal_handle.is_active

    def is_cancel_requested(self):
        return self.goal_handle and self.goal_handle.is_cancel_requested

    def is_executing(self):
        return self.goal_handle and self.goal_handle.status == GoalStatus.STATUS_EXECUTING

    def _execute_callback(self, goal_handle):
        self.logger.debug(f'{self} is beginning execution')
        self.goal_handle = goal_handle
        try:
            result = self.execute_callback(self.goal_handle.request)
        except Exception as e:
            result = None
            self.logger.error(
                f'An uncaught exception has occurred while trying to execute the action {self.info_string}: {e}')

        if result:
            self.goal_handle.succeed()
            return result
        else:
            self.goal_handle.abort()
            return self.action_type.Result()

    def __repr__(self):
        return f'SimpleActionServer({self.info_string})'
