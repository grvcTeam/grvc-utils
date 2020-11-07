//----------------------------------------------------------------------------------------------------------------------
// Mission lib
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2020 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <mission_lib.h>

class InheritanceTestClass : public grvc::mission_ns::Mission {
  public:
   InheritanceTestClass() {
       std::cout << " InheritanceTestClass running " << std::endl;
   }
   void printTest() {
       std::cout << test_ << std::endl;
   }

  protected:
   std::string test_ = "test";
};

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "mission_node");

    // InheritanceTestClass inheritance_test_class;    // Working just the same.
    // inheritance_test_class.print();
    // inheritance_test_class.printTest();

    // Takeoff WP parameters:
    geometry_msgs::PoseStamped takeoff_pose;
    takeoff_pose.pose.position.x = 100;
    takeoff_pose.pose.position.y = 0;
    takeoff_pose.pose.position.z = 20;
    float minimum_pitch = 0;

    // Pass WP parameters:
    std::vector<geometry_msgs::PoseStamped> pass_poses;
    geometry_msgs::PoseStamped pass_pose;
    pass_pose.pose.position.x = 250;
    pass_pose.pose.position.y = 0;
    pass_pose.pose.position.z = 30;
    pass_poses.push_back(pass_pose);
    pass_pose.pose.position.x = 0;
    pass_pose.pose.position.y = 250;
    pass_pose.pose.position.z = 30;
    pass_poses.push_back(pass_pose);
    pass_pose.pose.position.x = -250;
    pass_pose.pose.position.y = 0;
    pass_pose.pose.position.z = 30;
    pass_poses.push_back(pass_pose);
    pass_pose.pose.position.x = 0;
    pass_pose.pose.position.y = -250;
    pass_pose.pose.position.z = 30;
    pass_poses.push_back(pass_pose);
    pass_pose.pose.position.x = 250;
    pass_pose.pose.position.y = 0;
    pass_pose.pose.position.z = 30;
    pass_poses.push_back(pass_pose);
    pass_pose.pose.position.x = 100;
    pass_pose.pose.position.y = 0;
    pass_pose.pose.position.z = 30;
    pass_poses.push_back(pass_pose);

    // Loiter WP parameters:
    std::vector<geometry_msgs::PoseStamped> loiter_poses;
    geometry_msgs::PoseStamped loiter_pose;
    loiter_pose.pose.position.x = 250;
    loiter_pose.pose.position.y = 0;
    loiter_pose.pose.position.z = 30;
    loiter_poses.push_back(loiter_pose);
    loiter_pose.pose.position.x = 0;
    loiter_pose.pose.position.y = 250;
    loiter_pose.pose.position.z = 30;
    loiter_poses.push_back(loiter_pose);

    // Land WP parameters:
    geometry_msgs::PoseStamped loiter_to_alt_start_landing_pose;
    loiter_to_alt_start_landing_pose.pose.position.x = 200;
    loiter_to_alt_start_landing_pose.pose.position.y = 0;
    loiter_to_alt_start_landing_pose.pose.position.z = 20;
    geometry_msgs::PoseStamped land_pose;
    land_pose.pose.position.x = 0;
    land_pose.pose.position.y = 0;
    land_pose.pose.position.z = 0;

    grvc::mission_ns::Mission mission;
    std::cin.get();         // Wait for user input.
    mission.addTakeOffWp(takeoff_pose);
    // mission.addTakeOffWp(takeoff_pose, minimum_pitch);
    mission.addPassWpList(pass_poses);
    // mission.addLoiterWpList(loiter_poses, 20);
    // mission.addLoiterWpList(loiter_poses);
    mission.addLandWp(loiter_to_alt_start_landing_pose, land_pose); // For FIXED_WING
    // mission.addLandWp(land_pose);                                   // For VTOL and MULTICOPTER
    mission.print();
    mission.start();        // Still no mission pushed, so do nothing.
    std::cout << "active_waypoint_ = " << mission.activeWaypoint() << std::endl;    // Should give -1
    mission.push();
    std::cout << "active_waypoint_ = " << mission.activeWaypoint() << std::endl;    // Should give -1
    mission.start();        // Take off and start again (mission not cleared in the UAV).

    std::cin.get();         // Wait for user input.
    mission.start();        // Already flying, should do nothing.

    std::cin.get();         // Wait for user input.
    std::cout << "active_waypoint_ = " << mission.activeWaypoint() << std::endl;    // Should give whatever wp is doing.
    std::cin.get();         // Wait for user input.
    std::cout << "active_waypoint_ = " << mission.activeWaypoint() << std::endl;    // Should give whatever wp is doing.

    std::cin.get();         // Wait for user input.
    // If you wait until land, and arm or do "start", the mission will start again because the mission is still stored in the UAV.
    std::cout << "active_waypoint_ = " << mission.activeWaypoint() << std::endl;    // Should give -1
    mission.start();        // Take off and start again (mission not cleared in the UAV).

    // // Second mission to override the previous one:
    // std::cin.get();         // Wait for user input.
    // // mission.clear();
    // // mission.pushClear();    // Not needed if you are going to push another mission. Will "hold" current position (hovering if MULTICOPTER or orbit if VTOL or FIXED_WING) until new mission pushed.
    // // std::cin.get();         // Wait for user input.
    // mission.addTakeOffWp(takeoff_pose);
    // // mission.addTakeOffWp(takeoff_pose, minimum_pitch);
    // mission.addPassWpList(pass_poses);
    // // mission.addLoiterWpList(loiter_poses, 20);
    // // mission.addLoiterWpList(loiter_poses);
    // // mission.addLandWp(loiter_to_alt_start_landing_pose, land_pose); // For FIXED_WING
    // mission.addLandWp(land_pose);                                   // For VTOL and MULTICOPTER
    // mission.print();
    // mission.push();
    // mission.start();     // ALREADY FLYING! Will be ignored.

    while (ros::ok()) { sleep(1); }

    return 0;
}