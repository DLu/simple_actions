from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing

import unittest


def generate_test_description():
    integration_node = Node(
        package="simple_actions",
        executable="simple_actions_utest",
    )

    return (
        LaunchDescription(
            [
                integration_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"integration_node": integration_node},
    )


class TestGTestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_gtest_run_complete(self, proc_info, integration_node):
        proc_info.assertWaitForShutdown(integration_node, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_gtest_pass(self, proc_info, integration_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=integration_node)
