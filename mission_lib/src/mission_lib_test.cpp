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
    float acceptance_radius = 10;
    float orbit_distance = 0;

    // Land WP parameters:
    std::vector<geometry_msgs::PoseStamped> land_poses;
    geometry_msgs::PoseStamped land_pose;
    land_pose.pose.position.x = 100;
    land_pose.pose.position.y = 0;
    land_pose.pose.position.z = 20;
    land_poses.push_back(land_pose);
    land_pose.pose.position.x = 0;
    land_pose.pose.position.y = 0;
    land_pose.pose.position.z = 10;
    land_poses.push_back(land_pose);
    float loit_heading = 0;
    float loit_radius = 0;
    float loit_forward_moving = 1;
    float abort_alt = 0;
    float precision_mode = 0;

    grvc::mission_ns::Mission mission;
    std::cin.get();
    mission.addTakeOffWp(takeoff_pose, minimum_pitch);
    mission.addPassWpList(pass_poses, acceptance_radius, orbit_distance);
    mission.addLandWpList(land_poses, loit_heading, loit_radius, loit_forward_moving, abort_alt, precision_mode);
    mission.print();
    mission.push();
    mission.start();

    while (ros::ok()) { sleep(1); }

    return 0;
}