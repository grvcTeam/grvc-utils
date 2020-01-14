//----------------------------------------------------------------------------------------------------------------------
// GRVC SF11_DRIVER
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
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

class SF11Sim {
public:

    SF11Sim() {
        // Constant data
        data_.header.frame_id = "sf11";
        data_.radiation_type = sensor_msgs::Range::INFRARED;  // not really...
        data_.field_of_view = 0.0035;  // [rad]
        data_.min_range = 0.2;  // [m]
        data_.max_range = 120;  // [m]

        range_pub_ = n_.advertise<sensor_msgs::Range>("sf11", 1);
        scan_sub_ = n_.subscribe("sf11_sim_as_scan", 1, &SF11Sim::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        data_.header.stamp = msg->header.stamp;
        data_.range = msg->ranges[0];
        range_pub_.publish(data_);
    }

protected:
    sensor_msgs::Range data_;
    ros::NodeHandle n_;
    ros::Publisher range_pub_;
    ros::Subscriber scan_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sf11_sim_node");

	SF11Sim sf11_sim;
    ros::spin();

    return 0;
}
