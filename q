[33mcommit 84974860998e09406467d4bf26efb8ff5bd54d0b[m[33m ([m[1;36mHEAD -> [m[1;32mport_costmap_generator[m[33m, [m[1;31mupstream/port_costmap_generator[m[33m)[m
Merge: 0d37e54 326d286
Author: nik-tier4 <71747268+nik-tier4@users.noreply.github.com>
Date:   Mon Nov 2 18:41:29 2020 +0900

    Merge pull request #1 from mitsudome-r/port_costmap_generator
    
    fix subscription errors

[33mcommit 326d286898507c83909da053de971ca1fe9c5f6a[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Mon Nov 2 18:25:09 2020 +0900

    fix subscription errors
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 0d37e54dbf719f23499d7be741e0d30087fc549c[m
Author: Nithilan <nithilan.karunakaran@tier4.jp>
Date:   Mon Nov 2 16:55:53 2020 +0900

    porting in progress

[33mcommit 3ef8a4badb9ce2ec03f326a2e10e1df41c4e7718[m[33m ([m[1;31morigin/port_costmap_generator[m[33m)[m
Author: Nithilan <nithilan.karunakaran@tier4.jp>
Date:   Thu Oct 29 10:04:12 2020 +0900

    porting in progress

[33mcommit 057168b75bf650074d48b2abd24aa32ac8bfe98c[m
Author: Nithilan <nithilan.karunakaran@tier4.jp>
Date:   Wed Oct 28 10:39:30 2020 +0900

    working on porting changes in the code

[33mcommit f8672f944399b995d28ca1006097d33e503bec94[m
Author: Nithilan <nithilan.karunakaran@tier4.jp>
Date:   Tue Oct 27 13:49:01 2020 +0900

    ported CMakeLists.txt and package.xml to ROS2.
    
    dependency issues on grid_map_ros and grid_map_cv yet to be resolved.

[33mcommit 706089e468b9ed032ae779e8ee60e4fe8690a686[m[33m ([m[1;32mros2[m[33m)[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Mon Oct 26 13:34:25 2020 +0100

    Update build_and_test.yml (#57)

[33mcommit cc3da0f22552bb822192cd3dcc673b8e91061208[m
Author: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>
Date:   Fri Oct 23 15:30:16 2020 +0200

    Port osqp-interface to ros2 (#45)
    
    It doesn't use any ros functionality so the source code is unchanged, only `package.xml` and `CMakeLists.txt` needed to
    be modified. But osqp is added through a vendor package created at https://github.com/tier4/osqp_vendor
    
    To test this locally, eigen needs to be installed in the environment with
    
        sudo apt install libeigen3-dev

[33mcommit 4fed663e7f335eec812a02e30a23ae601a8517ba[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Fri Oct 23 22:25:13 2020 +0900

    port mission_planner to ROS2 (#56)
    
    * port mission_planner to ROS2
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix QoS for publishers
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 021fd68d7262b1fee00b7491a03a5c2195bca24d[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Fri Oct 23 15:03:09 2020 +0900

    add autoware_debug_msg (#54)
    
    * add autoware_debug_msg
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * rename to *stamped
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit b1714573b87a7187161f761588a0012a9ba029a3[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Fri Oct 23 07:51:43 2020 +0200

    Port autoware_api_msgs to ROS2 (#50)
    
    * Port autoware_api_msgs to ROS2
    
    * Format 3
    
    * Reviewer feedback

[33mcommit fe13902214ce7305f3aff7e21b51a86d127257f7[m
Author: Jilada Eccleston <jilada.eccleston@gmail.com>
Date:   Fri Oct 23 14:50:02 2020 +0900

    ROS2 Porting: gnss_poser (#18)
    
    * Fix CMakelist
     - Add ublox_msgs package dependency
     - Rename header files to use the .hpp extention
     - Remove colcon ignore
     - Fix Cmake ref to ublox in dependency
    
    * Fix compilation issues with convert.hpp
     - Add sensor_msgs::NavSatFix header which was missing before (?)
     - Remove logging for now
     - Contains warnings - geographic lib error polymorphous error
     - Fix geographic lib -Wcatch-value warnings
     - Add ifndef guard in gnss_stat.hpp
    
    * First pass - successful compilation
     - Fix CMakelist geographic library
     - Remove references to main ROS functions
     - Convert msg references
     - Add ifndef guards
     - Convert tf2 methods
     - Add functionality back
     - Fix type errors
    
    * Add publishers and subscribers
     - Add parameters
    
    * Convert launch files to ROS2 format
    
    * Add logging
    
    * Convert to component
    
    * Fix define guards to follow ROS2/Google style
    
    * Add -Werror compile flag
    
    * Clean up
    
    * Fix CMakelist to allow for the node to be found via ros2 launch
     - Revert filenames so it is easier to review
    
    * Address PR Comments:
     - Fix CMakelist and package.xml
     - Remove redundant depend/exec_depend calls
     - Remove redundant find_package calls
     - Explicitly add headers to the library
    
    * Address PR comments:
     - Revert removal of GNSSPoser namespace
     - Use std::make_shared in place of new
     - Reorder includes to conform with standard
    
    * Address PR Comments:
     - Rename variable to conform with the google style guide
     - Use the node clock in logging method calls
    
    * Address PR Comment:
     - Convert transform broadcaster to shared pointer
    
    * Address PR comment:
     - Remove -Werror compile flag post discussion

[33mcommit 3849e03b8e4237b4340963167d6d8205159ccac4[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Fri Oct 23 14:22:23 2020 +0900

    Update ci to use repos file (#43)
    
    * update ci to use repos file
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * add build_depends.repos
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 1504cb7c0fbfb011046ff1b917696c520459af79[m
Author: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>
Date:   Thu Oct 22 14:54:42 2020 +0200

    Remove vendor/osqp_vendor subtree (#55)

[33mcommit 63ceaabb6e68d5df162251c3d2190bff2eaac31d[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Thu Oct 22 10:38:32 2020 +0200

    Port pose_initializer to ROS2 (#11)
    
    * CMakeLists.txt & package.xml
    
    * Compiles
    
    * Works
    
    * Use callback instead
    
    * Reviewer feedback
    
    * Add sequence number check
    
    * Review feedback

[33mcommit f25cc0db5a215e20fb0d17fa603030cfe30d003a[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Thu Oct 22 13:08:24 2020 +0900

    Port lanelet2 extension (#36)
    
    * remove COLCON_IGNORE
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * port to ROS2
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * minor fix
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix CI
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * remove unnecessary semi-colon
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix library to executable for lanelet2_extension_sample and autoware_lanelet2_validation
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix usage for ROS2
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix usage message and parameter declaration
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix getting map_file parameter
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 2d57ef26e7fc230f5666d4ed87fa3e1c40d5cd80[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Wed Oct 21 19:46:05 2020 +0900

    port remote_cmd_converter (#49)
    
    * port remote_cmd_converter
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * port launch file to ROS2
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix parameter declarations
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix clock
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 2c75f3232e4976ecd569881f4c35477577704403[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Wed Oct 21 12:01:40 2020 +0200

    Port ndt to ROS2 (#29)
    
    * Port ndt to ROS2
    
    * Forgot COLCON_IGNORE
    
    * package.xml
    
    * Newline
    
    * Update CMakeLists.txt
    
    * Use ament_auto
    
    * Revert "Use ament_auto"
    
    This reverts commit 8007d92f6de20ba1128cda83476abe039d597374.
    
    * Use explicit targets

[33mcommit efbd2d64f381bdbb7555ce3b3f42a4c65928c213[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Wed Oct 21 17:38:35 2020 +0900

    port map_tf_generator (#32)
    
    * port map_tf_generator
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * add missing dependency
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix pointor, tf_broadcaster, add compile option
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * use ament_auto
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 2286a86082afc4973741d3f8a4dc6e1bbb3bd53f[m
Author: Jilada Eccleston <jilada.eccleston@gmail.com>
Date:   Wed Oct 21 15:04:04 2020 +0900

    ROS2 Porting: emergency_handler (#40)
    
    * Pull emergency handler package from master for porting
    
    * Fix CMakeList
     - Change to format similar to the simple_simulation_planner
     - Doesn't compile due to src files
    
    * Rename of core implementation to be compliant with ROS2 naming guidelines
    
    * Use the node/core naming convention
     - Add _core file to hold the implementation
     - _node file now holds the executable
     - Modify CMakeLists to point to the new executable
    
    * Conversion of msg types
     - Compiles now
     - Add back functions
    
    * Convert node intrinsics to ROS2
     - Add subscriptions
     - Add publisher
     - Add timer subscription
     - Add publishing and logging
    
    * Rearrange files and folders
    
    * Fix cmake and package xml
    
    * Interface with parameters and fix launch file
     - Fix configuration files
     - Correct main implementation
     - Clean up
     - Fix headers
    
    * Use correct timer implementation
    
    * Use throttle logs
    
    * Use class method to get time instead of static method
    
    * Make parameter file agnostic to node name
    
    Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
    
    Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>

[33mcommit 8a41390011185a95e19b33c6be7d6393ce2fef0d[m
Author: nik-tier4 <71747268+nik-tier4@users.noreply.github.com>
Date:   Wed Oct 21 12:31:48 2020 +0900

    ported pose2twist from ROS1 to ROS2 (#19)
    
    * ported pose2twist from ROS1 to ROS2
    
    * Update CMakeLists.txt
    
    updated CMakeLists to be consise using ament_auto
    
    * Update pose2twist_core.cpp
    
    updated the ~Pose2Twist() = default in header
    
    * Update package.xml
    
    sorted dependencies alphabetically
    
    * Update pose2twist_core.h
    
    added
      ~Pose2Twist() = default;
    
    * Update pose2twist_core.h
    
    Include guards replaced
    
    * edited pose2twist launch file to ROS2
    
    * fixed package XML

[33mcommit 7e59303ed2b0f699d33343429cc0c2c5683a4d2e[m
Merge: f53a05a 38e091a
Author: Esteve Fernandez <esteve@apache.org>
Date:   Tue Oct 20 17:10:35 2020 +0200

    Merge commit '38e091aa5ca210644e3bab0357e00ae53abebc11' as 'vendor/osqp_vendor'

[33mcommit 38e091aa5ca210644e3bab0357e00ae53abebc11[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Tue Oct 20 17:10:35 2020 +0200

    Squashed 'vendor/osqp_vendor/' content from commit 7bec512
    
    git-subtree-dir: vendor/osqp_vendor
    git-subtree-split: 7bec51212ce3c23c3e988372ee2e1a6885f84c33

[33mcommit f53a05ada584c031b0b702ec9e98b253dde26085[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Tue Oct 20 17:09:54 2020 +0200

    Revert "Squashed 'vendor/osqp_vendor/' content from commit 7bec512"
    
    This reverts commit d03e26df507c4595c281f02023ba0a75c5739183.

[33mcommit 271ef85cd68b59fdf1b76b7afa4ab49d1921526e[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Tue Oct 20 17:09:54 2020 +0200

    Revert "Added note about the origin of osqp"
    
    This reverts commit efe32c5113faee2f8143006e6907c5a73bf21259.

[33mcommit efe32c5113faee2f8143006e6907c5a73bf21259[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Tue Oct 20 16:55:09 2020 +0200

    Added note about the origin of osqp
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>

[33mcommit d03e26df507c4595c281f02023ba0a75c5739183[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Tue Oct 20 16:54:26 2020 +0200

    Squashed 'vendor/osqp_vendor/' content from commit 7bec512
    
    git-subtree-dir: vendor/osqp_vendor
    git-subtree-split: 7bec51212ce3c23c3e988372ee2e1a6885f84c33

[33mcommit ed797c533811068d0da564f2e27b9245c35dcc85[m
Author: Jilada Eccleston <jilada.eccleston@gmail.com>
Date:   Tue Oct 20 22:54:28 2020 +0900

    ROS2 Porting: gyro_odometer (#39)
    
    * Make compile
     - Fix cmake and package.xml
     - Update tf and msgs
    
    * Add publishers, subscribers and loggers
     - Fix launch file and Cmake to generate executable
    
    * Address PR comments:
     - Remove launch file xml line (not needed)
     - Convert Buffer and TransformListener to plain objects
     - Fix some typos

[33mcommit e092e3c715d853174bbd08c504c7c744fc29cdf2[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Tue Oct 20 15:52:15 2020 +0200

    Port autoware_utils (#33)
    
    * Port autoware_utils
    
    * Fix function name
    
    * Fix buildtool_depend

[33mcommit 2b992d436d6e2372d456d762d94ed366ba02868d[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Tue Oct 20 20:20:55 2020 +0900

    port shape_estimation to ros2 (#37)
    
    * port shape_estimation to ros2
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * minor fix from review
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * remove unused files
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * minor fix on include
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix cmakelist
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * remove unused codes
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix cmake
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 9a1d3f41cec7c1e6f9c846ea0b3599a7fd454265[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Tue Oct 20 07:53:54 2020 +0200

    Port ndt_omp to ROS2 (#20)
    
    * Remove COLCON_IGNORE
    
    * CMakeLists.txt & package.xml
    
    * Compiles
    
    * Remove rclcpp
    
    * Cleanup
    
    * Fix package.xml
    
    * Better CMakeLists.txt
    
    * Update CMakeLists.txt again
    
    * Update CMakeLists.txt
    
    * Update CMakeLists.txt yet again
    
    * Simplify CMakeLists.txt
    
    * Last (?) CMakeLists.txt change

[33mcommit dc7899d2cfda3894afa08d05cea0770b2a852e8e[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Fri Oct 16 12:38:10 2020 +0200

    Port ndt_pcl_modified to ROS2 (#21)
    
    * Port ndt_pcl_modified to ROS2
    
    * Newline
    
    * Better CMakeLists.txt
    
    * Add ament_cmake
    
    * Update CMakeLists.txt yet again
    
    * ament_auto
    
    * Revert "ament_auto"
    
    This reverts commit 53b785ef1d7f1d8f84c0c4a4a168c5c1c9a91a4a.

[33mcommit 17bf394b04070ad55605b32f87a3d2d5bb0fb328[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Thu Oct 15 22:15:10 2020 +0900

    set DEBIAN_FRONTEND=noninteractive after sudo (#31)
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 3f0ef23d7bd82e28b66365830b9bad164f358042[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Thu Oct 15 14:28:52 2020 +0200

    Rename launch files to launch.xml (#28)

[33mcommit fa154115f651006ee367fccfd82f46e8430ccb85[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Wed Oct 14 23:05:49 2020 +0900

    Port as pkg to ROS2 (#13)
    
    * add as ros2 pkg
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix launch
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * remove unused depend
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * as: modify timer callback
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * modify based on review
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix for review
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * add default param for global param
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * use ament_auto
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix for local param
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix cmake
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * remove unused, fix initialization order
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit c2c8a19c8ee25ff35fb717fdaa3102d0aa0e7b9e[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Oct 14 11:41:08 2020 +0200

    Port latlon_muxer to ROS 2 (#6)
    
    * Port CMakeLists.txt and package.xml
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * First cut at a port
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Port rest of the code
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Reformat code
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Removed COLCON_IGNORE por latlon_muxer
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Reenable stricter compile options
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Treat warnings as errors
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Use ament_auto
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Convert node to a component
    
    Signed-off-by: Esteve Fernandez <esteve@apache.org>
    
    * Made library shared

[33mcommit edca977d4e18d929fc3a395d6960e07dceefe3a7[m
Author: Jilada Eccleston <jilada.eccleston@gmail.com>
Date:   Wed Oct 14 17:40:03 2020 +0900

    Remove ublox from Pilot.Auto (#25)
    
    - Use Ublox forked from Teir4 until official ROS2 release

[33mcommit 480798bed5eaa2153dca594aff0cbbf2eae698c8[m
Author: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>
Date:   Wed Oct 14 10:38:17 2020 +0200

    Port shift decider to ros2 (#7)
    
    * Update package.xml and CMakeLists.txt to ros2
    
    Code doesn't compile yet
    
    * Code compiles
    
    * Update launch file
    
    * Add better timer
    
    clang-format shift_decider

[33mcommit e00e56c7befebc85f0c64f0665b190f13a7023d8[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Tue Oct 13 22:47:30 2020 +0900

    Fix simple planning simulator (#26)
    
    * simple planning simulator: fix params & launch file
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * remove unused file
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * fix timercallback
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 5eed6f21f8240003b073ba69a31a30104192fa47[m
Author: Nikolai Morin <nnmmgit@gmail.com>
Date:   Tue Oct 13 09:33:03 2020 +0200

    Port vehicle_cmd_gate to ROS2 (#3)
    
    * Port vehicle_cmd_gate to ROS2
    
    * Sim-time-respecting timer

[33mcommit b1030e4c028e4b3a2d54436d51e5045e183665ff[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Tue Oct 13 13:58:03 2020 +0900

    fix dependency issue in autoware_control_msgs (#22)
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 6b55b3bfc50b63b658a11cf196507370be6e8117[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Mon Oct 12 13:52:56 2020 +0900

    ROS2 Porting: raw_vehicle_cmd_converter (#9)
    
    * remove dependency to unused std_msgs package
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * rm COLCON_IGNORE
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * port raw_vehicle_cmd_converter to ROS2
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * port raw_vehicle_cmd_converter.launch to ROS2
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix typo
    
    Co-authored-by: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>
    
    * fix typo
    
    Co-authored-by: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>
    
    * remove include_directories form CMakeLists.txt
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * fix order of includes
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * add missing comments
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    Co-authored-by: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>

[33mcommit bfe938b7971f96b343cdf30e15b7157863804644[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Fri Oct 9 17:39:17 2020 +0900

    add apt update command to ci (#12)
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 5e19781c23827a3e202dd1ad0ffd354948aa8f78[m
Author: Jilada Eccleston <jilada.eccleston@gmail.com>
Date:   Fri Oct 9 17:38:21 2020 +0900

    ROS2 Porting: geo_pos_conv (#8)
    
    * Convert to ROS2 package
     - Update changelog -> 2.0.0
     - Update package xml to use the ROS2 xml schema
     - Update CMakelist with ament cmake commands
     - Add ROS2 compile options
    
    * Convert CMakelist for foxy
    
    * Clean up
     - Remove comments
    
    * Remove colcon ignore

[33mcommit 4d451881d2af9d001b90311c43ddaabb29d9f4e1[m
Merge: cd5c7e4 22fe078
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Thu Oct 8 17:36:49 2020 +0900

    Merge pull request #10 from mitsudome-r/revert-ros1
    
    Revert ros1

[33mcommit 22fe078869770d6cadd9b01f5e17d6dac17525db[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Thu Oct 8 17:27:14 2020 +0900

    add COLCON_IGNORE to ros1 packages
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 6a5d8ab390bf12c22e66b27668f86ed13b7ae6df[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Thu Oct 8 17:24:31 2020 +0900

    Revert "remove ROS1 packages temporarily"
    
    This reverts commit 60e838ee4c6ca9e2a2394fe9e8c81764539e07e9.
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit cd5c7e4af12c90cedc50f3f81a540d9d5e923cf9[m
Merge: bce7560 49cb72d
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Thu Oct 8 12:27:02 2020 +0900

    Merge pull request #4 from nnmm/autoware_localization_srvs
    
    Port autoware_localization_srvs

[33mcommit 49cb72dc48ed2f1e50269cdf488483eb1178a924[m
Author: Nikolai Morin <nikolai.morin@apex.ai>
Date:   Wed Oct 7 16:07:32 2020 +0200

    Add DEPENDENCIES

[33mcommit 72733ab7b4ab4a610bf7eb16183791b075733a20[m
Author: Nikolai Morin <nikolai.morin@apex.ai>
Date:   Wed Oct 7 14:07:13 2020 +0200

    Port autoware_localization_srvs

[33mcommit bce7560b4f241b84b0b884d208eb09420af54fc8[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Tue Oct 6 15:33:58 2020 +0900

    remove ROS1 packages
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 2ce46937c621e7c17daa6807d6dc1c5d65b29d54[m
Merge: 07bc1b0 48e9663
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Mon Oct 5 15:33:40 2020 +0900

    Merge pull request #2 from wep21/feature/ros2_perception_msgs
    
    Add ros2 perception msgs

[33mcommit 48e966342eeede977375488aacad2a95bdf880f0[m
Author: wep21 <border_goldenmarket@yahoo.co.jp>
Date:   Thu Oct 1 13:50:56 2020 +0900

    Add ros2 perception msgs
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 07bc1b06cf72473369c5926a776cf207743482f4[m
Merge: 217d183 adaec2c
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Thu Oct 1 16:12:55 2020 +0900

    Merge pull request #1 from esteve/ci
    
    Added CI

[33mcommit adaec2c66e51a378d9ca11ef88bb0ac04495a432[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Thu Oct 1 09:08:38 2020 +0200

    Only run for ros2-porting-test

[33mcommit 19c9f00821a04097ac9b485ded01e192babe5790[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 18:28:05 2020 +0200

    Use POSIX source

[33mcommit 58161dc05ed3ddefed153868ce725ec47732bab6[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 18:22:12 2020 +0200

    Source setup.bash

[33mcommit 5e25e22bc50aca1a098a2d471da17023aa66be22[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 18:00:03 2020 +0200

    Use a Docker image

[33mcommit b0363bf243564abdc7cc716927582e7138555f95[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 17:45:27 2020 +0200

    Use sudo

[33mcommit d0316b4c24d7a50d187be3853fe0bc6f4bdd5ba7[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 17:24:27 2020 +0200

    Source setup.bash

[33mcommit 2933bce4a4751d9160ee2672d5d7653aa6a13e79[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 17:17:54 2020 +0200

    Added from-paths

[33mcommit 20722e6304573e086cd8a48c8c2968f5c2bc1d29[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 16:34:18 2020 +0200

    Remove sudo rosdep init

[33mcommit 9dd0efdd52ea9d28fb044a6121f51812574655be[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 16:27:10 2020 +0200

    Add sudo rosdep init

[33mcommit 2340fd2647e4c413ccee432c2deffda60aeead01[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 16:21:18 2020 +0200

    Add sudo

[33mcommit fa0af69ae86827acaa161e9d8d1ef39eaf9be06f[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 15:58:34 2020 +0200

    Add rosdep update

[33mcommit 255ee5fed70c7c679f3beaf97a6ef9f2e706dce9[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 13:26:41 2020 +0200

    Run rosdep install

[33mcommit 92cf97ba093d9b23c0816eb07084e496e3ef9811[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 13:23:50 2020 +0200

    Use colcon directly

[33mcommit 6a61073ed9026f318de056703e8fc4e24a001c79[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 10:58:59 2020 +0200

    Added list of packages

[33mcommit 114f6e81d230be93d1b514bebbf6822c50160fa8[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 10:27:59 2020 +0200

    Do not pass a list of packages

[33mcommit d0b48eb3eb8344cc79aab868a6b616ff8ce26ad5[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 10:20:31 2020 +0200

    Check out repo

[33mcommit e412e97537509806d8fe5c7b3678fdde7ecec9ec[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 10:12:48 2020 +0200

    Use latest action-ros-ci action

[33mcommit 0aa41e37bdd4d9ad232b08bd057ed41103d9a8ba[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 09:58:41 2020 +0200

    Fix missing distro

[33mcommit 58c00acc83b1331465420e2b336dc24854cb1628[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 09:46:41 2020 +0200

    Switch to Ubuntu 20.04

[33mcommit 73c6d19a971444401409a4fb5a744979e1b130d0[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 09:40:25 2020 +0200

    Updates

[33mcommit 82bbd8c56b367b0a131833e6a92f17f8d4ff2474[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 09:37:27 2020 +0200

    Added test package

[33mcommit aa1e1ba6db62591632cc42560a6e419670347ff3[m
Author: Esteve Fernandez <esteve@apache.org>
Date:   Wed Sep 30 09:17:49 2020 +0200

    Added CI

[33mcommit 217d1833cffba32f70fcf89a28386bf328cdec45[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Tue Sep 29 19:22:55 2020 +0900

    add sample ros2 packages
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 60e838ee4c6ca9e2a2394fe9e8c81764539e07e9[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Tue Sep 29 19:22:18 2020 +0900

    remove ROS1 packages temporarily
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 8bcfbf7419552b5f6050e28c89c6488804652666[m[33m ([m[1;33mtag: v0.5.1[m[33m)[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Fri Sep 25 15:32:13 2020 +0900

    add license terms (#941)

[33mcommit e0d58f571a7d37f0ca0f57d2b8120a7e3721b6da[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Fri Aug 28 16:03:21 2020 +0900

    Fix/cublas dependency (#849)
    
    * fix cublas depencency
    
    * fix cublas depencency

[33mcommit e83ca2a6c49faeae267c60f1d72dfce2e26ef826[m[33m ([m[1;33mtag: v0.5.0[m[33m)[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Fri Aug 28 00:01:43 2020 +0900

    Fix velocity controller stopping (#830)
    
    * add debug code
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * cmd_vel = 0 when stopping
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * modify debug info
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit ff734b0c74b93f117bcb22fcf189b9518935f90c[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Thu Aug 27 16:51:53 2020 +0900

    Add lane_id in interpolated path point (#835)

[33mcommit 31317333fe6dffba355c018f690d3aa03a4d7196[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Wed Aug 26 19:47:47 2020 +0900

    Reduce calc cost stop planner (#833)
    
    * Reduce calc cost of slow down (#826)
    
    * Cosmetic change
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Fix rough search range
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Add circle inside/outside judgement
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Apply clang
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Parameterize step length
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Fix search range
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 250964dcb6732284db3b6aa6888f3b07767396b2[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 26 19:29:40 2020 +0900

    Fix/velocity controller (#836)
    
    * fix namespace
    
    * add new parameter to yaml
    
    * move stop dist param

[33mcommit bc2f250ad13c3bdaa433ed88547c18917d0c827f[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 26 16:34:58 2020 +0900

    Feature/add battery to awapi (#804)
    
    * implement battery fuel
    
    * update readme
    
    * add getGearInfo
    
    * fix message name
    
    * set default value:nan to battery

[33mcommit fd1e4065e4402d9afecf6e43a24b52d6c1914e64[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Wed Aug 26 11:56:18 2020 +0900

    Install executables in lanelet2_map_preprocessor (#834)
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 1f1c36f7e9097fa94f4e0e2358fdeb9dda0e8877[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Wed Aug 26 11:31:57 2020 +0900

    change raw pointer to vector and shared_ptr (#817)
    
    * change raw pointer to vector and shared_ptr
    
    * fix bug

[33mcommit cb134ec3b90818aa807a6692be1069c91d4b42b2[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Tue Aug 25 23:00:33 2020 +0900

     install launch when disable gpu (#829)
    
    * add launch and xml install when disable gpu
    
    * remove unnecessary install

[33mcommit 28038e5b9c4527a55900e9506354695b21411b49[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Tue Aug 25 22:11:32 2020 +0900

    Revert "Reduce calc cost of slow down (#826)" (#832)
    
    This reverts commit 845ab7b1b90f8817c8fcb12880b5af6f78c2b183.

[33mcommit 3c43fca00e5390132c3ae52364bb53f9e9a62354[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Tue Aug 25 19:27:53 2020 +0900

    Parameterization for calculating judge line distance (#831)
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 3d8b2558fec05678088aa3b40eef3f1d6b30238e[m
Author: Kazuki Miyahara <kmiya@outlook.com>
Date:   Tue Aug 25 13:32:55 2020 +0900

    fix uninitialized variables (#816)

[33mcommit dd5f6b14d5f4b8ee01425f05cca38fa10a81cd29[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Tue Aug 25 10:38:56 2020 +0900

    Reduce calc cost of slow down (#826)
    
    * Cosmetic change
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Fix rough search range
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Add circle inside/outside judgement
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Apply clang
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit ffd4422f26f9e8b3002adda6d5201a1d4a0073a5[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Mon Aug 24 17:41:43 2020 +0900

    Check distance to goal at lane change (#805)
    
    * Check distance to goal at lane change
    
    * Change calculation of distance to goal

[33mcommit 8cb1e66b9c656f8825c9d3a06e8ac198549df352[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Mon Aug 24 15:36:29 2020 +0900

    update trajectory visualizer (#818)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit b027a200817e78e9d1472c8100b7b74f4b0844ad[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Mon Aug 24 11:44:55 2020 +0900

    change default lowpass gain in acc (#820)
    
    * change default deceleration in acc
    
    * change lowpass gain

[33mcommit 924a263c4dd6049206ac8b5729e55c8f0c00e3b3[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Mon Aug 24 11:21:52 2020 +0900

    Check if lanelets are sequential (#823)
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 46f2f04dcfc6bae0ffc95fcbafaacf7463a6f6df[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Mon Aug 24 09:10:08 2020 +0900

    Fix/mpc sudden steer (#822)
    
    * mpc_follower: change order of calcActualVelocity (fix for sudden steer)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * mpc_follower: use current vel to calculate sign of curvature
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * mpc_follower: parametrize velocity dynamics info
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * mpc: fix path end time calc
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * mpc_follower: add description for extra time
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * mpc_follower: define param in yaml
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * mpc_follower: apply clang
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit efe1e81e111df5d54be03a78b5207758eef960c3[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Sun Aug 23 17:15:08 2020 +0900

    stopline: parametrize check distance (#825)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit d3ff0a7f86d80033c26c5f85ef4b0eb8e27de63f[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Sat Aug 22 23:37:07 2020 +0900

    change param (#821)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit d10124b406efedb707107c0cc8312341060c73a9[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Fri Aug 21 11:08:09 2020 +0900

    Fix/traffic light state (#801)
    
    * fix bug (passs judge line)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * bug fix
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * change param
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * bug fix : remove stop state
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * add lower limit
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 572255ca12e057f0d0caf17920a7a2623b939a97[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Fri Aug 21 10:41:19 2020 +0900

    velocity_controller: add keep stopping distance (#786)
    
    * velocity_controller: add keep stopping distance
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * avoid transition from pid to stop directly
    
    Co-authored-by: tomoya.kimura <tomoya.kimura@tier4.jp>

[33mcommit 0b2cd019e6148f26ad404a13456695c861402b6f[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Fri Aug 21 10:40:14 2020 +0900

    Revert "fix insert target point bug (#798)" (#815)
    
    This reverts commit 12eac2023538531e28a85a8914305cb534499f29.

[33mcommit eda4daf4f662215cb91afc181f12fef9308b0776[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 21 09:59:08 2020 +0900

    Feature/lowpass adaptive cruise control (#802)
    
    * add lowpass filter
    
    * update xml
    
    * change the method of insert velocity
    
    * change parameter

[33mcommit 4574bba04c633f9d4af9aedc248dc9b70eab5daa[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Fri Aug 21 03:41:27 2020 +0900

    Fix blind spot area (#806)
    
    * Fix blind spot area
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Use range based for
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Use std::unique to remove adjacent duplicates
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Use generateFineCenterline
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Fix detection area start point
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Revert "Use generateFineCenterline"
    
    This reverts commit 9d4f46bf65d7296b9286b30dfeed217a566a815c.
    
    * Fix centerline of half lanelet
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Fix area start point
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 4810d0d88b57405b1b9640c292ca62a0247c65bb[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Fri Aug 21 01:07:05 2020 +0900

    Fix intersection preceeding lane query (#807)
    
    * modified interseciton module to add lanelets in intersection to objective lanelets due to change in getPreceedingLaneletSequences()
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
    
    * update comment
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit 860f447fb8b09847d1abd07fd388dbcd46f44971[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Thu Aug 20 21:56:21 2020 +0900

    Feature/add stop reason lane change (#769)
    
    * add stop reason of lane change planner
    
    * add stop factor of blocked by obstacle
    
    * delete typo
    
    * add const
    
    * delete unnecessary namespace

[33mcommit c4d29f8d2919fb4de275c91ff4b06d8227b66fb0[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Thu Aug 20 20:50:56 2020 +0900

    Intersection add debug comment (#803)
    
    * blind spot : modify debug comment
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * merge from private road : modify debug comment
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * intersection : modify debug comment
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * intersection util: modify generateStopLine() for debug
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * add comment
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 5586e3428f45cbef55b03fb264cf480ecc26be82[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Thu Aug 20 19:32:01 2020 +0900

    traffic light scene module : fix bug and change param (#800)
    
    * fix bug (passs judge line)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * bug fix
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * change param
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit da00558998fb324e51102300135811c15da903b7[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Thu Aug 20 19:12:22 2020 +0900

    Feature/change output awapi (#782)
    
    * change ouput of awapi
    
    * change error output to warn

[33mcommit f1f77b5893532092bb4c604c58ca5828ab59b2ca[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Thu Aug 20 13:13:49 2020 +0900

    fix typo stop_liine to stop_line (#799)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 668a981e23b6c043f244b21f58a8520c7859caa3[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Thu Aug 20 12:39:55 2020 +0900

    fix insert target point bug (#798)

[33mcommit 6fcd94e132eb022639703f3fcf83c21ffa9db459[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Wed Aug 19 21:13:51 2020 +0900

    Remove duplicated param (#797)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit e84a9bf0b5b81a212b8c5c66783dbce18b76e761[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Wed Aug 19 20:17:35 2020 +0900

    Change min_slow_down_vel to 3kmph (#796)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit a7baa5030d79d877376cd47a18f9275322b41233[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Wed Aug 19 17:17:04 2020 +0900

    Improve autoware state monitor diag (#792)
    
    * Add ok_list to stats
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Use key-value
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Add ok_list
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Fix format
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 2b7b4d6f9914d63c709b2122f799e7a0e93fedc7[m
Author: Satoshi Tanaka <st14.828soccer@gmail.com>
Date:   Wed Aug 19 17:16:18 2020 +0900

    update parameter (#793)

[33mcommit 0764f99738bdac14da8e99bc7c98ae9c70ec5e59[m
Author: Kosuke Murakami <kosuke.murakami@tier4.jp>
Date:   Wed Aug 19 16:28:47 2020 +0900

    Fix/extend drivable area beyond goal (#781)
    
    * update llt2 extention query func
    
    Signed-off-by: Kosuke Murakami <kosuke.murakami@tier4.jp>
    
    * extend drivable area over goal point
    
    Signed-off-by: Kosuke Murakami <kosuke.murakami@tier4.jp>
    
    * apply clang
    
    Signed-off-by: Kosuke Murakami <kosuke.murakami@tier4.jp>
    
    * update get preeceeding func
    
    Signed-off-by: Kosuke Murakami <kosuke.murakami@tier4.jp>
    
    * update preceeding func in lanechange
    
    Signed-off-by: Kosuke Murakami <kosuke.murakami@tier4.jp>
    
    * update comment
    
    Signed-off-by: Kosuke Murakami <kosuke.murakami@tier4.jp>

[33mcommit 864e8fac9e2534279cc62130a2021ce2cce38cd6[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Wed Aug 19 15:16:05 2020 +0900

    change margin to 1.0m (#790)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 7546bccb08d482c34aabd7e5abda38b8460d4151[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 19 14:45:48 2020 +0900

    Feature/smooth adaptive cruise (#789)
    
    * change param
    
    * do not insert max velocity near the ego vehicle position

[33mcommit 358cac5293cb96935c695758552522ca8dfa7e7d[m
Author: ito-san <57388357+ito-san@users.noreply.github.com>
Date:   Wed Aug 19 12:50:38 2020 +0900

    Added the mode of CPU Usage to check statistics calculated as averages among all processors by default. (#788)

[33mcommit f66654ef800a3aa36c8a30a5f31486befbb04fcd[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Wed Aug 19 10:00:51 2020 +0900

    fix wrong start index (#745)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 0f245866e91431af74651dffc025de4a95409d7f[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Wed Aug 19 09:49:30 2020 +0900

    add obstacle_stop_planner.yaml (#766)
    
    * add stop_planner.yaml
    
    * change file name
    
    * add explanation of the parameters

[33mcommit 3bd4055ceca3637b642fbdaea5c59af839b7a7e2[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Wed Aug 19 09:48:58 2020 +0900

    Add arg for input tl topic name (#783)
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit f3597007d9088906cb9787daac32f34f7d593598[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Wed Aug 19 01:16:22 2020 +0900

    add dead line and pass through when over dead line (#784)

[33mcommit 12324a822d9231c13c5e35cc8dff9b42970b352b[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Wed Aug 19 00:06:21 2020 +0900

    Add surround obstacle state (#785)
    
    * Cleanup code
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Add state to surround_obstacle_checker
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Guard pushObstaclePoint for StopFactor
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit c0f94ae80c038f3e31ad516d495acdd632b402a3[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Tue Aug 18 23:40:57 2020 +0900

    add resolution param in lanelet2_extension (#760)

[33mcommit e8dacf792c54ce09535b366ffa78322683ef283f[m
Author: ito-san <57388357+ito-san@users.noreply.github.com>
Date:   Tue Aug 18 17:54:34 2020 +0900

    Fixed various bugs. (#768)
    
    * Fixed various bugs.
    
    * Fixed wrong status report of NIC.

[33mcommit c6af112b6579d94e2ce50a8720444b8941b8acbb[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Aug 18 16:31:59 2020 +0900

    fix lane change bug (#780)
    
    * add exception handling
    
    * fix exception handling
    
    * clean code

[33mcommit 3b6b19975b698da26dc7fe4719ca246cff23d2e8[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Aug 18 16:07:19 2020 +0900

    avoid to input -1 index (#777)

[33mcommit 7b2349969b15c16bc237a2fbf64a544f450a3cfb[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Aug 18 16:05:43 2020 +0900

    fix pred vel in negative velocity (#779)

[33mcommit 4bc667205a2f7c61282b972276c0017e0d404a98[m
Author: hiroyuki obinata <58019445+obi-t4@users.noreply.github.com>
Date:   Tue Aug 18 10:26:59 2020 +0900

    update awapi readme (#747)

[33mcommit ea1582e1a58d00cf9fe02d7af3a6245d3838ff9c[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Aug 18 08:59:20 2020 +0900

    fix parameter (#778)

[33mcommit 797c857feb05579f594b469ae18c6066555eb482[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Mon Aug 17 16:08:13 2020 +0900

    Fix/intersection stuck detect area (#764)
    
    * fix stuck-vehicle detection area in intersection module
    
    * separate vehicle param
    
    * change the way ouf start idx count
    
    * change param

[33mcommit 449ce085ca75bbf8cd6cb0e4ed13931fc298ed2b[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Mon Aug 17 14:49:07 2020 +0900

    Fix blind spot areas and logic for creating a stop line (#732)
    
    * Fix blind spot areas
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Change logic for creating a stop line
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Do not add marker when marker is empty
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Cosmetic change
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Return boost optional for first conflict point
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Return boost optional for intersection start point
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Apply clang
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>

[33mcommit f7939d96f28d354d9a177a0de1465311350e1f89[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Mon Aug 17 13:48:27 2020 +0900

    Fix routing from crosswalk (#767)
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>

[33mcommit 79f60611d53ef8e34441b98623f844e172e8c3bf[m
Author: Kosuke Takeuchi <kosuke.tnp@gmail.com>
Date:   Sun Aug 16 23:13:51 2020 +0900

    Fix typo cliped -> clipped (#776)
    
    Signed-off-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>

[33mcommit 89d5581c36e0a47cfeaca74ac9df4aa0931527d3[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Sat Aug 15 05:15:11 2020 +0900

    Fix/publish tl state stamped (#772)
    
    * Publish tl states stamped (#744)
    
    * Add TrafficLightStateStamped.msg
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Use msg instead of struct
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Publish traffic light state
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Check if lamp state is empty
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Apply clang-format
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Cosmetic change
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Use dynamic pointer cast
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>

[33mcommit 5811325c7a49dd46f1c1b13b326cc08f7f3c7ed2[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 14 21:24:49 2020 +0900

    change uid of marker (#775)

[33mcommit d62063f20eef9a6ff09f5cf3d90edf7a448b77e6[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 14 19:11:54 2020 +0900

    fix intersection bug (#773)
    
    * fix bug
    
    * fix bug

[33mcommit d0d52051c99d909063a01988dc36e205f376d374[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Fri Aug 14 19:06:05 2020 +0900

    Fix acceleration constraint (#774)
    
    * Fix acceleration constraint
    
    * Apply clang-format

[33mcommit 99e8be415ec3f9c89de9e21fc48f6e398b4bd005[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 14 16:57:55 2020 +0900

    add accel/brake offset (#757)

[33mcommit b4fc19b89be536fdfd796d3e80c48ba87f09244a[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 14 16:28:57 2020 +0900

    fix lerp by timestamp (#770)

[33mcommit 053b936ed476a7bb62fe9e8c67665a54e3638d64[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Fri Aug 14 16:05:04 2020 +0900

    Revert "Publish tl states stamped (#744)" (#771)
    
    This reverts commit 35a7e29e987afb29d9348b5f64866a584a65f753.

[33mcommit c73d9576d21294c355d2668e9feebdeaf7d5e885[m
Author: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
Date:   Fri Aug 14 14:40:41 2020 +0900

    fix bug in getLaneChangePaths (#750)
    
    Signed-off-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>

[33mcommit e32218c934d5d7f9c50b2a4fca06d7612033951f[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 14 13:45:26 2020 +0900

    fix stop factor of merge_from_private_area (#749)

[33mcommit 6bf404502f717b0322bf911d748470956b737e39[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Thu Aug 13 16:58:13 2020 +0900

    dont check pointcloud (#765)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 60c44564730b991f71b3a5e4216a5cc5fb14354b[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Thu Aug 13 11:04:57 2020 +0900

    remove libutils dependency when unable gpu (#761)

[33mcommit befde3f1c978b3744d6e320522aa982771427f37[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 12 19:18:15 2020 +0900

    fix stack area (#758)

[33mcommit 16d9d12ecf4e6369bab18ee880b84f468807ff6f[m
Author: ito-san <57388357+ito-san@users.noreply.github.com>
Date:   Wed Aug 12 19:05:26 2020 +0900

    Fixed uninitialized variable. (#763)

[33mcommit e81ea0bf7aa4084b4bcdec17664307c5acd51613[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 12 18:34:52 2020 +0900

    update visualization marker (#759)

[33mcommit be2a1df88d2ddffec721f05b4be64d813fb3cfd7[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Wed Aug 12 18:15:45 2020 +0900

    Fix/ring outlier filter bug (#762)
    
    * fix bug, and check azimuth
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>
    
    * reduce calc cost
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * fix bug
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * change default param
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    Co-authored-by: Yamato Ando <yamato.ando@gmail.com>

[33mcommit fb68e9b29779ce4ac175040c360da46c05a219b7[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 12 18:12:19 2020 +0900

    Fix/velocity controller (#753)
    
    * apply jerk filter after slope filter
    
    * do not predict velocity when velocity is low
    
    * prevent miss-prediction of velocity

[33mcommit 649a4376965c7dd103b177b94427b0f2ca3a393f[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 12 12:32:35 2020 +0900

    Feature/extract leaf diag info (#741)
    
    * add mock function of extract leaf diag
    
    * remove parent diag
    
    * change method of isLeaf
    
    * clean code
    
    * change varibale/function name
    
    * remove string predefine
    
    * update readme

[33mcommit 6406e7be8deb7d69a92ad4f24962fcb1b67c12b8[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Wed Aug 12 12:30:35 2020 +0900

    Add missing dependencies of autoware_joy_controller (#755)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 3b1f68b58cc6b3e1d8f31d9fd3492c43dd7f20e9[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Wed Aug 12 00:15:25 2020 +0900

    Publish tl states stamped (#744)
    
    * Add TrafficLightStateStamped.msg
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Use msg instead of struct
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Publish traffic light state
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Check if lamp state is empty
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Apply clang-format
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Cosmetic change
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>

[33mcommit 955718748692767ea4250c2184119a8e68665b3d[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Tue Aug 11 16:58:14 2020 +0900

    Fix direct trans to stopped (#752)
    
    * velocity_controller: prohibit direct transition from PID to STOPPED without going through SMOOTH_STOP
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>
    
    * velocity_controller: rename controller_mode to control_mode
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 21e815a7082d45aac2825386df25a3596962ed99[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Tue Aug 11 00:48:40 2020 +0900

    intersection: ignore stop plane when path[0] is in detection area (#739)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit fcb41c428ae209dee6c1f97714bce89d8e0107b6[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Aug 7 22:58:14 2020 +0900

    Fix/acc same pointcloud (#743)
    
    * add handling with same pointcloud
    
    * clean code

[33mcommit 77b4bca3983e854d1cb6642cd0584a520e8d32f2[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Fri Aug 7 20:59:39 2020 +0900

    cosmetic change (#738)
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 816f04b4a3a35b5c54feed957b2603f6b4293e1f[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Fri Aug 7 20:09:36 2020 +0900

    Add warning when vehivle will not engage (#746)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 40363247926f9f98342101ba93676410da8f2f51[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Thu Aug 6 19:42:27 2020 +0900

    add debug values of adaptive cruise control (#742)
    
    * add debug values of adaptive cruise control
    
    * add rqt config file

[33mcommit b1871a7b112e3bc0d56ef499eb3b28e0c1d1f667[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Thu Aug 6 16:23:12 2020 +0900

    Change key map for G29 controller and set deadzone parameter (#740)

[33mcommit 9e8d32845cee9828cf91b659e96b2cf8aaee7078[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Thu Aug 6 10:40:13 2020 +0900

    Feature/add goal api (#734)
    
    * add  goal api
    
    * add msg
    
    * update readme
    
    * add  const auto
    
    * clean code

[33mcommit 3410311f73ba687b9b1c83461131aebd4745df2f[m
Author: hiroyuki obinata <58019445+obi-t4@users.noreply.github.com>
Date:   Thu Aug 6 00:53:13 2020 +0900

    modify put/goal -> put/route (#736)

[33mcommit 31df8984e7aaffadeee4feee8477ca9dc93a3cd5[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Wed Aug 5 21:57:02 2020 +0900

    Remove override after arrived goal in autoware_state_monitor (#737)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 4be53c216bfc3fd0553c6d9c8bb2d5bbe9d919d1[m
Author: Kosuke Murakami <kosuke.murakami@tier4.jp>
Date:   Wed Aug 5 21:53:35 2020 +0900

    Feature/improve obstacle avoidance (#731)

[33mcommit 5b1726a3f1025e7989f812ff8b773df6ac4d975b[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Wed Aug 5 20:52:34 2020 +0900

    Include enable_slope_compensation to yaml file (#735)

[33mcommit d34c7a77ca93cd90ba1431d65bf37bbb9d0a1f0e[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Aug 5 16:56:34 2020 +0900

    Feature/stop reason api (#729)
    
    * fix indent of readme
    
    * add mock of stop reason aggregator
    
    * add constptr to stop reason
    
    * aggregate message
    
    * add empty handling
    
    * add nullptr handling
    
    * fix bug
    
    * update readme
    
    * fix bug
    
    * remove unused ifdef
    
    * unify same reason msg to stop factor

[33mcommit 423f09b388ab7613f190f5aa1a02136b9f8786c1[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Wed Aug 5 15:40:07 2020 +0900

    fix clac signed distance bug (#733)

[33mcommit f879b17abc0c6e5e1a1cfddef559e733302bfba7[m
Author: Satoshi Tanaka <st14.828soccer@gmail.com>
Date:   Wed Aug 5 14:26:52 2020 +0900

    add warning logging to foa.data.is_is_avoidance_possible (#725)

[33mcommit 2c575693c88b808e7e2a899e33046ab1edd29c90[m
Author: Kosuke Murakami <kosuke.murakami@tier4.jp>
Date:   Wed Aug 5 09:36:22 2020 +0900

    update osqp interface (#730)

[33mcommit 67fa7a7ae8613446cdfd2f23a98f58f36cd0ae6c[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Tue Aug 4 20:06:55 2020 +0900

    Fix/spline interpolation in intersection module (#726)
    
    * Remove duplicating sample points
    
    * Change isValidInput
    
    * Apply clang-format
    
    * Fix convergence check in PCG

[33mcommit 0c7cea1403dd2e223d87624e243d5a539e041d70[m
Author: YamatoAndo <yamato.ando@gmail.com>
Date:   Tue Aug 4 17:33:07 2020 +0900

    ring_outlier_filter: fix bug, and check azimuth (#727)
    
    * fix bug, and check azimuth
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>
    
    * reduce calc cost
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 796f030276939f63563cdf4baf3e91ad4aa21b16[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Tue Aug 4 17:24:17 2020 +0900

    Support error in autoware state (#728)
    
    * Support Error in autoware_state_monitor
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Refactor autoware_state_monitor
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Wait after planning completed
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Integrate FailedToArriveGoal and Error into Emergency state
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit e7e12dc5c7c74d27b52c52fbaa7f3ef1da6c9e82[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Tue Aug 4 11:29:01 2020 +0900

    Calc acc in velocity controller (#716)
    
    * Calculate acceleration in velocity_controller
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Add acc to multiplot
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit bca124dbe2efb16ede8298ddaf5dc5f908fb4e4b[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Mon Aug 3 16:22:07 2020 +0900

    Fix/stop reason (#724)
    
    * input stop reason of traffic light
    
    * add comment
    
    * add empty traffic light handling
    
    * change calculation method of traffic light position
    
    * avoid 0 position output

[33mcommit a1f0880e03e4d781ad9928766fd8535ce6105a2d[m
Author: hiroyuki obinata <58019445+obi-t4@users.noreply.github.com>
Date:   Mon Aug 3 15:48:14 2020 +0900

    add check mark for awapi readme (#723)
    
    * add check mark for awapi readme
    
    * add blank
    
    * add blank
    
    * add blank

[33mcommit e5f2125af4588f745c8c0c42a40eeb7f1f24cd30[m
Author: Akihito Ohsato <aohsato@gmail.com>
Date:   Mon Aug 3 12:54:44 2020 +0900

    Modify to use projection matrix with undistorted 2D result (#722)
    
    Signed-off-by: Akihito Ohasto <aohsato@gmail.com>

[33mcommit 4d83919171001a9da7112a2226fb37392530bf3c[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Mon Aug 3 11:52:59 2020 +0900

    Feature/stop reason (#712)
    
    * add stop reason msg
    
    * add mock of stop resaon publisher
    
    * change namespace of stop reason
    
    * update stop reason msg
    
    * add toRosPoint
    
    * implement stop reason publisher of blind stop
    
    * implement stop reason publisher of crosswalk
    
    * implement stop reason publisher of intersection
    
    * implement stop reason publisher of stop line
    
    * implement stop reason publisher of trafficlight
    
    * implement stop reason publisher of detection area
    
    * fix bug
    
    * remove unnecessary process
    
    * add remained stop factor
    
    * clean code
    
    * fix bug
    
    * not punlish stop reason if array size is 0
    
    * add stop reason to stuck object in intersection
    
    * add stop factor of obstacle stop planner
    
    * add stop reason of surround_obstacle checker
    
    * Apply review
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * fix message type
    
    * delete unused message from cmake
    
    * remove stopReasonStamped
    
    * change topic name of stop reasons
    
    Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 40c2d976e8e1253e1db22125973dc3a6aeda12b6[m
Author: Daichi Murakami <harihitode@gmail.com>
Date:   Mon Aug 3 00:03:46 2020 +0900

    Avoid setting CMAKE_BUILD_TYPE=Release in each CMakeLists.txt (#720)
    
    * remove set CMAKE_BUILD_TYPE Release in each CMakeLists.txt
    
    * remove set CMAKE_BUILD_TYPE Release in ndt_pcl_modified
    
    * set compile options for debug in ndt_omp
    
    * Fix indent
    
    * add warning if -DCMAKE_BUILD_TYPE=Release is not set in ndt_omp
    
    Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>

[33mcommit 4ea10d3fef7140b886863fea7b36342264978f70[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Fri Jul 31 14:24:08 2020 +0900

    fix typo (#721)

[33mcommit 51c176afe745b30c50b98f983b46014cb138f175[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Thu Jul 30 11:24:44 2020 +0900

    Add nodelets of tlr nodes (#715)
    
    * Add classifier nodelet
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Replace boost::shared_ptr into std::shared_ptr
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Add lock guard
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Add detetcor nodelet
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Integrate main into node
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Add SubscriberStatusCallback
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * add image_transport_decompresser nodelet
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * Add visualizer nodelet
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * fixed bug
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * Fix plugin name
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Launch nodelet
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * Fix classifier constructor
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * add decompresser node
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * fix typo
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * fixed bug
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * fixed bug
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * cosmetic change
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * add param
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * fix bug
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * Fix build warning
    
    Signed-off-by: Daisuke Nishimatsu <border_goldenmarket@yahoo.co.jp>
    
    * change rgb
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * change rgb
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 96f837462aa31154957fc325285b84bd44ea34d4[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Thu Jul 30 10:28:10 2020 +0900

    Revert "bug fix and reduce calc cost (#708)" (#719)
    
    This reverts commit e69564a4071a0849435ac8ffe54fe4e49d8e3974.

[33mcommit c80f922a895993a78beb9dfe73be017833acf637[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Thu Jul 30 10:26:15 2020 +0900

    Change localization diag namespace (#718)
    
    * Change default value of output_diagnostics_topic
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Remove localization_diagnostic.js from web_controller
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Integrate localization diag to diagnostic_aggregator
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Delete old and unused publisher
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 4c4f1d5e847285e94ef3c37b7fa10af0744cd409[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Wed Jul 29 21:42:40 2020 +0900

    fixed bug (#717)

[33mcommit 45519dba686d0a038732273be2f3206de6583680[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Wed Jul 29 15:54:28 2020 +0900

    Feature/intersection use spline interpolation library (#710)
    
    * Use spline_interpolation module
    
    * Remove debug messages

[33mcommit f49afb4e3bfe1a5cd8d4a078f3bcc054a2a914f0[m
Author: YamatoAndo <yamato.ando@gmail.com>
Date:   Wed Jul 29 15:26:10 2020 +0900

    add control mode status (#709)
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>

[33mcommit 2e59bb6708898b80c1583d3a87929f0d32b64762[m
Author: YamatoAndo <yamato.ando@gmail.com>
Date:   Tue Jul 28 19:12:13 2020 +0900

    rename gnss package (#714)
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>

[33mcommit 8b17e3c66324b8252c8a9d44c687c525cd56b6bf[m
Author: Yukihiro Saito <yukky.saito@gmail.com>
Date:   Tue Jul 28 19:07:05 2020 +0900

    bug fix and reduce calc cost (#708)
    
    * bug fix and reduce calc cost
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>
    
    * fixed bug
    
    Signed-off-by: Yukihiro Saito <yukky.saito@gmail.com>

[33mcommit 9d4091786e011ac607331fd772c0ff07b0ce820c[m
Author: YamatoAndo <yamato.ando@gmail.com>
Date:   Tue Jul 28 12:06:30 2020 +0900

    Fix/concat mutex bug (#711)
    
    * modify defautl value
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>
    
    * fix bug
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>
    
    * fix mutex bug
    
    Signed-off-by: Yamato Ando <yamato.ando@gmail.com>

[33mcommit af0fc18598c15542dc0d5741cdea6826ac373f11[m
Author: Satoshi Tanaka <st14.828soccer@gmail.com>
Date:   Mon Jul 27 17:12:52 2020 +0900

    fix apply dynamic param (#704)

[33mcommit e8609eb7eace0cc37c6dc37be631c18c0bc32026[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Sat Jul 25 14:20:22 2020 +0900

    Fix/awapi awiv adapter (#698)
    
    * subscribe global rpt
    
    * update readme
    
    * update readme
    
    * add warning
    
    * fix readme
    
    * update readme

[33mcommit 91326f8fdf065f5e470105d3a997fff12e23e6a9[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Wed Jul 22 20:40:42 2020 +0900

    Add spline interpolation library (#705)

[33mcommit c8412526bdb30df62e678f324ac87805baca28a8[m
Author: Daichi Murakami <harihitode@gmail.com>
Date:   Wed Jul 22 07:11:32 2020 +0900

    check if gdown command exists (#707)

[33mcommit 8d0b1d650f5461855cc613bc1918029358cb7b0f[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Jul 21 23:36:27 2020 +0900

    Fix/steer simulator (#697)
    
    * fix topic/param name
    
    * fix indent
    
    * fix steer simulator
    
    * add xml
    
    * Format .launch
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Format .py
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Add wait_for_param
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit f4c3fca33a46f902b060e5d80b6c92de23e89332[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Tue Jul 21 20:28:54 2020 +0900

    add use_object_recognition flag in dummy_perception_publisher (#696)

[33mcommit 8acb83e2c1dc05962bc7f9b0879f2e21a6ea5657[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Tue Jul 21 20:12:10 2020 +0900

    add convertToXYZCloud (#706)

[33mcommit 2b7d8155725b4fb356ac5f308780c4e1b9728acd[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Mon Jul 20 14:29:37 2020 +0900

    Change default value of disengage_on_route (#703)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 21c6e1336d2226a2af7dab8f543f4933f02a0b9b[m
Author: Satoshi Tanaka <st14.828soccer@gmail.com>
Date:   Mon Jul 20 13:20:41 2020 +0900

    add dynamic_reconfigure to  velocity_controller (#694)
    
    * add dynamic_reconfigure to  velocity_controller
    * refactoring

[33mcommit d7798e3266233175d2e66e78d71ffc072fe2ffea[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Sun Jul 19 17:59:58 2020 +0900

    intersection: fix stuck vehicle behavior (#695)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 03be557d21d15ed6ec01174c6a285eb587486d09[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Sun Jul 19 17:59:19 2020 +0900

    Support G29 controller in autoware_joy_controller (#699)
    
    * Add map for G29 controller
    
    * Add new line at end of file
    
    * Change structure of JoyConverterBase class
    
    * Rename PS4 -> DS4
    
    * Rename controler_type -> joy_type
    
    * Set joy_type by console input
    
    * Change doc
    
    * Remap g29 controller
    
    * Remap AccelPedal -> accel, BrakePedal -> brake
    
    * Remove [autoware_joy_controller] from ROS_INFO
    
    Co-authored-by: Fumiya Watanabe <fumiya.watanabe@tier4.jp>

[33mcommit d60f2c7909b39d307367da5d7d331960cbb91e8e[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Fri Jul 17 17:29:09 2020 +0900

    intersection: change detection area length parameter from 100m to 200m (#702)
    
    Signed-off-by: Takamasa Horibe <horibe.takamasa@gmail.com>

[33mcommit 91f3fb23cfa3c3a48f4667fef2e6947b88d030cd[m
Author: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
Date:   Fri Jul 17 17:28:32 2020 +0900

    Fix engage cmd when override is active (#684)
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 5be77a8fc48104438a67ccb4c7a10092ee3b5fae[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Fri Jul 17 15:09:07 2020 +0900

    add dynamic_reconfigure to obstacle avoidance planner (#673)

[33mcommit bd87108f34ff90a53efa340ce1d90836cd18754c[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Fri Jul 17 11:23:30 2020 +0900

    Fix detection area (#701)
    
    * Replace calcDistance with calcSignedDistance
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>
    
    * Add STOP state
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit 3879952605449780668029c383d3a6bb207732d5[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Fri Jul 17 01:13:30 2020 +0900

    Fix typo (#700)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit d4f5db78476a8ef101828db77dc0ddf0da989b4a[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Thu Jul 16 10:59:35 2020 +0900

    Devide raw_vehicle_converter.launch from pacmod_interface.launch (#683)
    
    Co-authored-by: Fumiya Watanabe <fumiya.watanabe@tier4.jp>

[33mcommit e0eddfb614736ad6188673c9f1aa031d48a186a9[m
Author: Taichi Higashide <taichi.higashide@tier4.jp>
Date:   Thu Jul 16 00:07:24 2020 +0900

    add accel and decel setter to optimizer (#692)
    
    * add accel and decel setter to optimizer
    
    * add scoped lock in run method
    
    * devide setter
    
    * boost -> std

[33mcommit 56c1acfe294f3bcadf304bfe0448d1f1eba2ee55[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Wed Jul 15 19:11:12 2020 +0900

    move msg file to autoware_api_msgs (#693)

[33mcommit 80a249183d87494858e0e54ad914bc040d18e303[m
Author: Fumiya Watanabe <rej55.g@gmail.com>
Date:   Wed Jul 15 13:48:17 2020 +0900

    Change topic name and change plot time length to 90 sec. (#691)
    
    Co-authored-by: Fumiya Watanabe <fumiya.watanabe@tier4.jp>

[33mcommit 63ad7475760d00bf8896e62e721db729b246e052[m
Author: shin <8327162+0x126@users.noreply.github.com>
Date:   Tue Jul 14 20:05:14 2020 +0900

    add velocity_controller_param_path (#690)
    
    Signed-off-by: Shinnosuke Hirakawa <shinnosuke.hirakawa@tier4.jp>

[33mcommit b31f6cc1fa49052c41017197f9bb8d19c0c4e966[m
Author: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
Date:   Tue Jul 14 20:00:00 2020 +0900

    Add map diag (#688)
    
    Signed-off-by: Kenji Miyake <kenji.miyake@tier4.jp>

[33mcommit a9463c1b0a16d7ce21b01677377a51d82b112daa[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Tue Jul 14 19:44:36 2020 +0900

    ssc_interface : add /vehicle/stauts/turn_sitgnal pub (#689)
    
    Signed-off-by: TakaHoribe <horibe.takamasa@gmail.com>

[33mcommit 69f15230c96799c301451582acd75ac99f03a3c0[m
Author: wep21 <42202095+wep21@users.noreply.github.com>
Date:   Tue Jul 14 18:09:25 2020 +0900

    Add quick stop mode when changing gear (#666)
    
    * Add quick stop mode when changing gear
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Cosmetic change
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>
    
    * Apply clang format
    
    Signed-off-by: wep21 <border_goldenmarket@yahoo.co.jp>

[33mcommit 85faab9537081304b185709d090e15e536436a83[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Jul 14 16:48:55 2020 +0900

    fix typo (#687)

[33mcommit 9da33a35f76ef02830cc3be83aa992640816b930[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Jul 14 14:38:16 2020 +0900

    fix bug(return true in getPose) (#686)

[33mcommit be95cbf167513f1ca7bb6569bb1d43a9c2b66fa9[m
Author: tkimura4 <tomoya.kimura@tier4.jp>
Date:   Tue Jul 14 13:56:11 2020 +0900

    Feature/awapi awiv adapter first pr (#685)
    
    * add base file of awapi_awiv_adapter
    
    * publish autoware status
    
    * update readme
    
    * fix readme
    
    * rename file
    
    * add relay topic
    
    * change msg name
    
    * publish autoware status
    
    * update readme
    
    * add new message
    
    * add lane change topic, obstacle avoidance topic
    
    * update readme
    
    * fix readme
    
    * add namespace
    
    * rename lane change available
    
    * fix readme
    
    * change pub hz
    
    * change topic name
    
    * change control mode and add gate_mode
    
    * fix readme
    
    * update readme

[33mcommit 117304f545eeea5b653f799dd3df98d9e0b1e18c[m
Author: Takamasa Horibe <horibe.takamasa@gmail.com>
Date:   Mon Jul 13 10:23:53 2020 +0900

    pacmod_interface : add /vehicle/status/turn_signal pub (#627)
    
    * pacmod_interface : add /vehicle/status/turn_signal pub
    
    Signed-off-by: TakaHoribe <horibe.takamasa@gmail.com>
    
    * add indent
    
    Signed-off-by: TakaHoribe <horibe.takamasa@gmail.com>

[33mcommit 5dd029c2f6df6cb5a5bc05d0e31797460e7dc63a[m
Author: UMiho <58927122+UMiho@users.noreply.github.com>
Date:   Thu Sep 24 16:55:40 2020 +0900

    Create LICENSE

[33mcommit 30f2ca0a82e836e2d787264326af6cf6c832d4b2[m[33m ([m[1;33mtag: v0.4.0[m[33m)[m
Author: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
Date:   Fri Sep 18 15:10:43 2020 +0900

    release v0.4.0
