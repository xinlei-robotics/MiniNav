"""The stock, GCC-built loaders load our Clang-built libraries that import MiniNav modules.

- rclcpp_components' component_container loads ModuleProbeNode, which parses the robot.yaml
  installed by mininav_core and logs it;
- nav2_controller's controller_server creates StopController through pluginlib when it is
  configured.
"""

import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState


@pytest.mark.launch_test
def generate_test_description():
    container = ComposableNodeContainer(
        name='toolchain_check_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mininav_toolchain_check',
                plugin='mininav_toolchain_check::ModuleProbeNode',
                name='module_probe',
            ),
        ],
        output='screen',
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'controller_frequency': 20.0,
            'controller_plugins': ['FollowPath'],
            'FollowPath': {'plugin': 'mininav_toolchain_check::StopController'},
        }],
    )
    return (
        launch.LaunchDescription([container, controller_server, launch_testing.actions.ReadyToTest()]),
        {'container': container, 'controller_server': controller_server},
    )


class TestStockLoaders(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('toolchain_check_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_component_container_loads_the_probe(self, proc_output, container):
        proc_output.assertWaitFor('robot.yaml from mininav_core:', process=container, timeout=30)

    def test_controller_server_creates_the_plugin(self, proc_output, controller_server):
        client = self.node.create_client(ChangeState, '/controller_server/change_state')
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.assertTrue(future.done(), 'configure transition timed out')
        # A second controller_server in the same ROS domain would answer too (and refuse to
        # configure twice): run the test in a quiet domain.
        self.assertTrue(future.result().success,
                        'controller_server failed to configure '
                        '(is another controller_server running in this ROS domain?)')
        proc_output.assertWaitFor(
            'Created controller : FollowPath of type mininav_toolchain_check::StopController',
            process=controller_server, timeout=10)
