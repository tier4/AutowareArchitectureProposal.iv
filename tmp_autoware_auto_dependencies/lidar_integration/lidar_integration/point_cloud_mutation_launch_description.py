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


def get_point_cloud_mutation_launch_description(*,
                                                test_nodes,
                                                checkers,
                                                topic="points_nonground",
                                                other_actions=[],
                                                spoofer=None):

    # Other nodes/procesess in the test fixture:
    if spoofer is None:
        spoofer = launch_ros.actions.Node(
            package="lidar_integration",
            executable="point_cloud_mutation_spoofer_exe",
            arguments=[
                "--topic", topic,
                "--freq", "10",
                "--runtime", "60",
                "--mean", "300000",
                "--std", "50000",
            ],
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
        *test_nodes
    ])

    return ld, {
        "_checkers": checkers,
        "_spoofer": spoofer,
        "_test_nodes": test_nodes,
    }


def make_active_mutation_tests():

    class TestWaitForEnd(unittest.TestCase):
        def test_exits(self, proc_info, _checkers, _spoofer):
            # Wait until spoofer is done and checkers have finished, spoofer
            # runs for 60s
            proc_info.assertWaitForShutdown(process=_spoofer, timeout=120)
            for checker in _checkers:
                proc_info.assertWaitForShutdown(process=checker, timeout=120)

    return TestWaitForEnd


def make_post_shutdown_mutation_tests():
    @launch_testing.post_shutdown_test()
    class TestCheckerOutput(unittest.TestCase):
        def assert_processes_exit_code(self, proc_output, proc_info, processes, exit_code):
            # Also check the exit code of the checker as a belt-and-suspenders
            # approach
            for proc in processes:

                # For diagnosing problems with the test, it's helpful to have
                # the checker output for passing and failing runs
                if proc in proc_output:
                    for line in proc_output[proc]:
                        print(line.text.decode(), end='')

                self.assertEqual(exit_code, proc_info[proc].returncode)

        def test_checker_outputs_success(self, proc_output, _checkers):
            for checker in _checkers:
                launch_testing.asserts.assertInStdout(proc_output, "success", checker)

        def test_test_nodes_exit_code_ok(self, proc_output, proc_info, _test_nodes):
            self.assert_processes_exit_code(proc_output, proc_info, _test_nodes, exit_code=0)

        def test_checker_exit_code_ok(self, proc_output, proc_info, _checkers):
            self.assert_processes_exit_code(proc_output, proc_info, _checkers, exit_code=0)

        def test_spoofer_exit_code_ok(self, proc_output, proc_info, _spoofer):
            self.assert_processes_exit_code(proc_output, proc_info, [_spoofer], exit_code=0)

    return TestCheckerOutput
