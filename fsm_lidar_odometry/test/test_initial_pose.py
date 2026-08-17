# Copyright 2022 Alexandros Filotheou
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
What the node makes of the pose it is asked to start from.

The service waits for one message on the initial pose topic and folds the
heading it carries into every pose the node goes on to publish. A quaternion of
four zeros describes no rotation at all, and the arithmetic that reads a
heading out of one divides by its own length. Whether that yields a value that
is not a number or a heading of zero is up to the compiler, and neither is an
answer: one is in every pose from then on and no later scan clears it, the
other is a bearing the operator never asked for. The pose has to be refused.

These cases get a node of their own. The service holds whichever node serves it
until a message arrives on the topic, so a case that drives it must not leave
behind a node that the next case expects to answer promptly.
"""

import math
import time
import unittest

from fsm_lidar_odometry_test_support import (
    build_scan,
    FIRST_STAMP,
    Harness,
    SECOND_STAMP,
    SERVICE_TIMEOUT,
    THIRD_STAMP,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from std_srvs.srv import Trigger


def pose_without_a_heading():
    """
    Return a pose message whose orientation is four zeros.

    ROS 2 fills an untouched quaternion field with the identity rotation, so
    the zeros have to be written out. They arrive all the same: from a bridge
    carrying ROS 1 traffic, where an untouched quaternion is all zeros; from a
    message deserialised out of a buffer nobody wrote to; and from anybody
    assembling a pose by hand who sets three components and leaves the fourth.
    """
    message = PoseWithCovarianceStamped()
    message.pose.pose.orientation.x = 0.0
    message.pose.pose.orientation.y = 0.0
    message.pose.pose.orientation.z = 0.0
    message.pose.pose.orientation.w = 0.0
    return message


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='fsm_lidar_odometry',
        executable='fsm_lidar_odometry_interface_node',
        name='fsm_lidar_odometry',
        output='screen',
    )
    return (
        launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]),
        {'fsm_lidar_odometry': node},
    )


class TestInitialPose(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = Harness()
        self.harness.wait_for_connection()
        self.harness.spin(0.5)

    def tearDown(self):
        self.harness.destroy_node()

    def offer(self, message):
        """
        Call set_initial_pose and keep offering the pose until it answers.

        The node only subscribes once the service has been called, and the
        topic is not latched, so a pose published beforehand is long gone by
        the time anybody is listening for it.
        """
        client = self.harness.service_clients['set_initial_pose']
        self.assertTrue(client.wait_for_service(timeout_sec=SERVICE_TIMEOUT))

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + SERVICE_TIMEOUT

        while not future.done() and time.monotonic() < deadline:
            self.harness.initial_pose_publisher.publish(message)
            rclpy.spin_once(self.harness, timeout_sec=0.1)

        self.assertTrue(future.done(), 'set_initial_pose never answered')
        return future.result()

    def test_a_pose_carrying_no_heading_is_refused(self):
        response = self.offer(pose_without_a_heading())

        self.assertFalse(response.success,
                         'an all zero quaternion was taken as a heading')
        self.assertTrue(response.message,
                        'the refusal said nothing about what was wrong')

    def test_a_pose_carrying_no_heading_never_reaches_a_published_pose(self):
        self.offer(pose_without_a_heading())

        self.harness.call('start')
        self.harness.clear()

        for stamp, pose in ((FIRST_STAMP, (3.0, 2.5, 0.0)),
                            (SECOND_STAMP, (3.02, 2.5, 0.0)),
                            (THIRD_STAMP, (3.04, 2.5, 0.0))):
            self.harness.publish(build_scan(stamp, pose))
            self.harness.spin()

        self.assertGreater(len(self.harness.poses), 0,
                           'the node published no pose to examine')

        for published in self.harness.poses:
            position = published.pose.position
            orientation = published.pose.orientation
            for name, value in (('position.x', position.x),
                                ('position.y', position.y),
                                ('orientation.x', orientation.x),
                                ('orientation.y', orientation.y),
                                ('orientation.z', orientation.z),
                                ('orientation.w', orientation.w)):
                self.assertTrue(
                    math.isfinite(value),
                    'a refused initial pose reached %s as %r' % (name, value))

    def test_a_pose_that_describes_a_rotation_is_taken(self):
        message = PoseWithCovarianceStamped()
        message.pose.pose.position.x = 3.0
        message.pose.pose.position.y = 2.5
        message.pose.pose.orientation.w = 1.0

        response = self.offer(message)

        self.assertTrue(response.success, response.message)


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
