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
Behaviour of the running node, independent of whether its numbers are right.

Whether the matcher computes the right answer is settled elsewhere, against the
reference build. What is checked here is the surface the node presents: that
the services do what they say, that frames and stamps come out as documented,
and that malformed input is refused rather than crashing.

Every case shares one node, so each begins by putting it into a known state.
The properties that only a never started node can show live in the startup
test, which gets a node of its own.
"""

import math
import time
import unittest

from fsm_lidar_odometry_test_support import (
    build_scan,
    FIRST_STAMP,
    FOURTH_STAMP,
    Harness,
    SECOND_STAMP,
    SERVICE_TIMEOUT,
    THIRD_STAMP,
)
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import ReliabilityPolicy


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


class TestNodeBehaviour(unittest.TestCase):

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

        # One node serves every case here, so put it somewhere known first.
        self.harness.call('stop')
        self.harness.call('clear_estimated_trajectory')
        self.harness.clear()

    def tearDown(self):
        self.harness.destroy_node()

    def drive(self, poses, stamps):
        for stamp, pose in zip(stamps, poses):
            self.harness.publish(build_scan(stamp, pose))
            self.harness.spin()

    def test_a_stopped_node_ignores_scans(self):
        self.drive([(3.0, 2.5, 0.0), (3.02, 2.5, 0.0)],
                   [FIRST_STAMP, SECOND_STAMP])

        self.assertEqual(self.harness.odometry, [])
        self.assertEqual(self.harness.poses, [])

    def test_starting_resumes_output_and_stopping_halts_it_again(self):
        self.harness.call('start')
        self.harness.clear()
        self.drive([(3.0, 2.5, 0.0), (3.02, 2.5, 0.0)],
                   [FIRST_STAMP, SECOND_STAMP])
        self.assertGreater(len(self.harness.odometry), 0)

        response = self.harness.call('stop')
        self.assertTrue(response.success)
        self.assertTrue(response.message)

        self.harness.clear()
        self.harness.publish(build_scan(THIRD_STAMP, (3.04, 2.5, 0.0)))
        self.harness.spin()
        self.assertEqual(self.harness.odometry, [])

        self.assertTrue(self.harness.call('start').success,
                        'the node did not survive being stopped')

    def test_clear_estimated_trajectory_empties_the_path(self):
        self.harness.call('start')
        self.drive([(3.0, 2.5, 0.0), (3.02, 2.5, 0.0), (3.04, 2.5, 0.0)],
                   [FIRST_STAMP, SECOND_STAMP, THIRD_STAMP])
        self.assertGreater(len(self.harness.paths[-1].poses), 1)

        response = self.harness.call('clear_estimated_trajectory')
        self.assertTrue(response.success)
        self.assertTrue(response.message)

        self.harness.clear()
        self.harness.publish(build_scan(FOURTH_STAMP, (3.06, 2.5, 0.0)))
        self.harness.spin()

        self.assertEqual(len(self.harness.paths[-1].poses), 1,
                         'the path was not emptied')

    def test_output_is_stamped_from_the_scan_that_produced_it(self):
        self.harness.call('start')
        self.harness.clear()
        self.drive([(3.0, 2.5, 0.0), (3.02, 2.5, 0.0)],
                   [FIRST_STAMP, SECOND_STAMP])

        self.assertGreater(len(self.harness.odometry), 0)

        stamp = self.harness.odometry[-1].header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), SECOND_STAMP)

        stamp = self.harness.poses[-1].header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), SECOND_STAMP)

    def test_frames_are_as_configured_and_carry_no_leading_slash(self):
        self.harness.call('start')
        self.harness.clear()
        self.drive([(3.0, 2.5, 0.0), (3.02, 2.5, 0.0)],
                   [FIRST_STAMP, SECOND_STAMP])

        odometry = self.harness.odometry[-1]
        self.assertEqual(odometry.header.frame_id, 'lo')
        self.assertEqual(odometry.child_frame_id, 'base_laser_link')
        self.assertEqual(self.harness.poses[-1].header.frame_id, 'map')

        transform = self.harness.transforms[-1].transforms[0]
        self.assertEqual(transform.header.frame_id, 'lo')
        self.assertEqual(transform.child_frame_id, 'base_laser_link')

        for name in (odometry.header.frame_id, odometry.child_frame_id,
                     self.harness.poses[-1].header.frame_id):
            self.assertFalse(name.startswith('/'), name)

    def test_a_scan_of_a_different_length_is_matched_rather_than_dropped(self):
        """
        The first scan settles the size and later scans are resampled to it.

        No size is configured by default, so the first scan's own resolution
        is what the session matches at. A driver that occasionally truncates a
        scan should cost one coarser match, not a gap in the odometry.
        """
        self.harness.call('start')
        self.harness.publish(build_scan(FIRST_STAMP, (3.0, 2.5, 0.0)))
        self.harness.spin()
        self.harness.clear()

        self.harness.publish(build_scan(SECOND_STAMP, (3.02, 2.5, 0.0), size=180))
        self.harness.spin()
        self.assertEqual(len(self.harness.odometry), 1,
                         'a scan of a different length produced no output')

        self.harness.publish(build_scan(THIRD_STAMP, (3.04, 2.5, 0.0)))
        self.harness.spin()
        self.assertEqual(len(self.harness.odometry), 2,
                         'the node did not carry on afterwards')

    def test_parameters_take_their_documented_defaults(self):
        expected = {
            'scan_topic': '/base_scan',
            'global_frame_id': 'map',
            'base_frame_id': 'base_laser_link',
            'lo_frame_id': 'lo',
            'size_scan': 0,
            'min_magnification_size': 0,
            'max_magnification_size': 3,
            'num_iterations': 2,
            'xy_bound': 0.2,
            'max_counter': 200,
            'max_recoveries': 10,
            'rng_seed': 0,
            'ray_search': 'angular',
            'scan_qos_reliability': 'reliable',
            'scan_qos_depth': 1,
        }

        client = self.harness.create_client(
            GetParameters, '/fsm_lidar_odometry/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=SERVICE_TIMEOUT))

        request = GetParameters.Request()
        request.names = list(expected)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.harness, future,
                                         timeout_sec=SERVICE_TIMEOUT)

        values = future.result().values
        self.assertEqual(len(values), len(expected))

        for name, value in zip(expected, values):
            wanted = expected[name]
            if isinstance(wanted, bool):
                self.assertEqual(value.bool_value, wanted, name)
            elif isinstance(wanted, int):
                self.assertEqual(value.integer_value, wanted, name)
            elif isinstance(wanted, float):
                self.assertAlmostEqual(value.double_value, wanted, msg=name)
            else:
                self.assertEqual(value.string_value, wanted, name)

    def test_the_scan_subscription_is_reliable_by_default(self):
        deadline = time.monotonic() + 20.0
        subscription = None

        # Nothing else in this test subscribes to the scan topic, so whatever
        # turns up there is the node under test.
        while subscription is None and time.monotonic() < deadline:
            found = self.harness.get_subscriptions_info_by_topic('/base_scan')
            if found:
                subscription = found[0]
            rclpy.spin_once(self.harness, timeout_sec=0.1)

        self.assertIsNotNone(subscription,
                             'the node does not appear among the subscribers')
        self.assertEqual(subscription.qos_profile.reliability,
                         ReliabilityPolicy.RELIABLE)

    def test_two_scans_sharing_a_stamp_report_no_velocity(self):
        """
        A repeated stamp is ordinary input and must not yield an infinity.

        A replayed recording is free to carry the same stamp twice, and a
        driver that stamps on publication rather than on acquisition can emit
        two inside one clock tick. The displacement between the pair is still
        measured and is still published. The velocity is not, because there is
        no elapsed time to divide it by.
        """
        self.harness.call('start')
        self.harness.publish(build_scan(FIRST_STAMP, (3.0, 2.5, 0.0)))
        self.harness.spin()
        self.harness.clear()

        self.drive([(3.02, 2.5, 0.0), (3.04, 2.5, 0.0)],
                   [SECOND_STAMP, SECOND_STAMP])

        self.assertEqual(len(self.harness.odometry), 2,
                         'the two scans did not both produce odometry')

        moving = self.harness.odometry[0].twist.twist
        self.assertNotEqual(
            (moving.linear.x, moving.linear.y, moving.angular.z),
            (0.0, 0.0, 0.0),
            'the pair with time between them reported no velocity either, so '
            'this case would prove nothing about the pair without')

        odometry = self.harness.odometry[-1]
        twist = odometry.twist.twist

        for name, value in (('linear.x', twist.linear.x),
                            ('linear.y', twist.linear.y),
                            ('angular.z', twist.angular.z)):
            self.assertTrue(math.isfinite(value),
                            'twist %s came out as %r' % (name, value))
            self.assertEqual(value, 0.0, name)

        position = odometry.pose.pose.position
        self.assertTrue(math.isfinite(position.x),
                        'the measured displacement was dropped as well')
        self.assertTrue(math.isfinite(position.y),
                        'the measured displacement was dropped as well')


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
