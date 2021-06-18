# Copyright 2021 The Autoware Foundation
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
# Developed by Robotec.ai

# Add a smoke test
# :param package_name: name of the package to smoke test
# :type package_name: string
# :param package_exec: package executable to run during smoke test
# :type package_exec: string

function(add_smoke_test package_name package_exec)
  add_ros_test(
    ${autoware_testing_DIR}/../autoware_testing/smoke_test.py
    TARGET "smoke_test"
    ARGS "arg_package:=${package_name}" "arg_package_exe:=${package_exec}"
    TIMEOUT "30"
  )
endfunction()
