# Copyright 2019 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import unittest

import launch
import launch.actions
import launch_ros.actions

import launch_testing.event_handlers


def get_lidar_launch_description(*,
                                 test_nodes,
                                 checkers,
                                 other_actions=[],
                                 port="21345",
                                 spoofer=None):
    """
    Set up a launch description that can be used to run lidar integration tests.

    :param test_nodes: An array of launch.actions that execute the nodes being tested

    :param checkers: An array of launch.actions representing the checker processes that will test
    the ROS ouput of the test_nodes

    Checkers can be created using the lidar_integration.make_pcl_checker factory function.  One
    checker is expected for each topic to be checked

    :param other_actions: An optional list of other actions to be launched along with the checker
    actions

    :param port: The port number to use for the UDP spoofer.  Tests should use
    lidar_ingetration.get_open_port() so as not to conflict with other tests that may run at the
    same time

    :param spoofer: To override the default vlp16_integration_spoofer (for the velodyne driver).
    Compliant spoofers must write "Spoofer(s) number is:' to the console when they start to
    output data

    :returns: A tuple of (launch.LaunchDescription, test_context dictionary).  The test_context
    dictionary must be part of the test_context returned by the parent generate_test_description
    function.  Its members are considered private and are not expected to be used by the parent
    test description function
    """
    # Other nodes/procesess in the test fixture:
    if spoofer is None:
        spoofer = launch_ros.actions.Node(
            package="lidar_integration",
            executable="vlp16_integration_spoofer_exe",
            arguments=[
                "--rpm", "600",
                "--ip1", "127.0.0.1",
                "--ip2", "127.0.0.1",
                "--port1", "{}".format(port),
            ]
        )

    ld = launch.LaunchDescription([
        # Make sure the spoofer is up and running before launching the checker processes
        launch.actions.RegisterEventHandler(
            launch_testing.event_handlers.StdoutReadyListener(
                target_action=spoofer,
                ready_txt='Spoofer(s) number is:',
                actions=checkers + other_actions
            )
        ),

        # Add action to execute spoofer
        spoofer,

        # Add actions for test nodes
        *test_nodes,
    ])

    return ld, {
        "_checkers": checkers,  # An array of all the checkers to be enumerated by the tests
        "_spoofer": spoofer,
        "_test_nodes": test_nodes,
    }


def make_active_tests():

    class TestWaitForEnd(unittest.TestCase):
        def test_checker_exits(self, proc_info, _checkers):
            # The checker node exits automatically after about 10 seconds (specified in the launch)
            for checker in _checkers:
                proc_info.assertWaitForShutdown(process=checker, timeout=60)

    return TestWaitForEnd


def make_post_shutdown_tests():
    @launch_testing.post_shutdown_test()
    class TestCheckerOutput(unittest.TestCase):
        def test_checker_outputs_success(self, proc_output, _checkers):
            for checker in _checkers:
                launch_testing.asserts.assertInStdout(proc_output, "success", checker)

        def test_checker_exit_code_ok(self,
                                      proc_output,
                                      proc_info,
                                      _checkers,
                                      _spoofer,
                                      _test_nodes):
            print("Spoofer output:")
            for line in proc_output[_spoofer]:
                print(line.text.decode(), end='')

            print("Test node outputs:")
            for nd in _test_nodes:
                for line in proc_output[nd]:
                    print(line.text.decode(), end='')

            # Also check the exit code of the checker as a belt-and-suspenders approach
            for checker in _checkers:

                # For diagnosing problems with the test, it's helpful to have
                # the checker output for passing and failing runs
                if hasattr(checker, "_checker_id"):
                    print("Checker {}".format(checker._checker_id))

                for line in proc_output[checker]:
                    print(line.text.decode(), end='')

                self.assertEqual(0, proc_info[checker].returncode)

    return TestCheckerOutput
