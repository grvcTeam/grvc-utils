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

#ifndef MISSION_LIB_H
#define MISSION_LIB_H

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace grvc {

enum struct AutopilotType {PX4, APM, DJI, UNKNOWN};
enum struct AirframeType {FIXED_WING, MULTICOPTER, VTOL, OTHER, UNKNOWN};

class Mission {

public:

    Mission();
    Mission(int _uav_id, const std::string& _node_name_space="");
    Mission(int _uav_id, const std::string& _pose_frame_id, const std::string& _node_name_space);
    Mission(const Mission& _mission);
    Mission& operator=(const Mission& _mission) { return *this; }
    void constructorFunction();

    ~Mission();

    // Get latest pose estimation of the robot:
    geometry_msgs::PoseStamped pose();
    sensor_msgs::NavSatFix geoPose() const { return cur_geo_pose_; }

    // Get latest velocity estimation of the robot:
    geometry_msgs::TwistStamped velocity() const { return cur_vel_; }

    // Get latest battery estimation of the robot:
    float battery() const { return battery_percentage_; }

    // Getters for the autopilot and airframe type:
    AutopilotType autopilotType() const { return autopilot_type_; };
    AirframeType airframeType() const { return airframe_type_; };

    // Getter for knowing if the UAV is armed or disarmed:
    bool armed() const { return armed_; };

    // Get current waypoint of the list that define de mission (-1 if not running a mission or disarmed):
    int activeWaypoint() const { return active_waypoint_; };

    // Getter for the uav identifier (may be redundant, but useful for debug):
    int id() const { return robot_id_; };

    // Set home or launch site at (0,0,0) in the pose() output:
    bool setHome(bool _set_z);

    // IMPORTANT: Right now the map origin is considered to be always at z=0, which makes sense, and all the MAVROS waypoints will be done just fine. But the initial pose of the UAV (the one where mission_lib is constructed) is considered to be also at z=0, which may not be always correct.
    // The only consecuence of this is that pose() will give an output with an offset in z if the initial z of the drone isn't 0 meters. You can solve this issue by uncommenting the "TODO X" in "mission_lib.cpp", but you will have to tune both map_origin_geo[2] and cur_geo_pose_.altitude so that are correct.

    // Functions to build, delete, print, push, start and stop the mission:
    void addTakeOffWp(const geometry_msgs::PoseStamped& _takeoff_pose, float _minimum_pitch=15);    // For FIXED_WING try that the pose is far enough straight to the direction of the plane. For VTOL and MULTICOPTER any point is valid.
    void addPassWpList(const std::vector<geometry_msgs::PoseStamped>& _pass_poses, float _speed=-1, float _acceptance_radius=10, float _pass_radius=0);     // Add simple waypoint where the UAV will pass. _speed=-1 for default speed.
    void addLoiterWpList(const std::vector<geometry_msgs::PoseStamped>& _loiter_poses, float _time=-1, float _radius=75, float _speed=-1, float _turns=-1, float _forward_moving=0, float _heading=-1);     // Add loiter waypoint where the UAV will wait or "hold" an amount of time defined by the user. _speed=-1 for default speed.
    void addLandWp(const geometry_msgs::PoseStamped& _land_pose, float _abort_alt=0, float _precision_mode=0);      // Landing for VTOL and MULTICOPTER
    void addLandWp(const geometry_msgs::PoseStamped& _loiter_to_alt_start_landing_pose, const geometry_msgs::PoseStamped& _land_pose, float _langing_yaw, float _loit_radius=60, float _loit_heading=1, float _loit_forward_moving=1, float _abort_alt=0, float _precision_mode=0); // Landing for FIXED_WING (needed an extra point for where to start the landing and reach the desired landing height, recommended far enough from the actual landing site).
    void print() const; // Print in the terminal the local mission in the Mission class (not the one pushed to the UAV).
    void clear();       // Clear the local mission in the Mission class (not the one pushed to the UAV).
    bool push();        // Overwrite the mission in the UAV with the one local one. If already flying the mission will start automatically (ignoring the takeoff waypoint).
    bool pushClear();   // Clear the mission in the UAV. If flying it will "hold" current position (hovering if MULTICOPTER, orbit if VTOL or FIXED_WING) until new mission pushed.
    void start();       // Take off and start mission if there is a mission pushed to the UAV, if there is no mission do nothing. If already flying and another mission is pushed it's not needed to do "start" again, it will start automatically when pushed (ignoring the takeoff waypoint).
    void stop();        // Change flight mode into auto return to launch (AUTO.RTL) and land directly the UAV using the landing waypoints existing on the mission.

private:
    void addSpeedWp(float _speed);  // Change the horizontal speed of the UAV. Private because, for the sake of simplicity, it's recommended to change it direcly with addPassWpList and addLoiterWpList.

    void getAutopilotInformation();
    void initHomeFrame();
    void setFlightMode(const std::string& _flight_mode);
    void arm(bool _arm);

    void setParam(const std::string& _param_id,int _param_value);
    double updateParam(const std::string& _param_id);
    std::map<std::string, double> mavros_params_;

    std::vector<geographic_msgs::GeoPoseStamped> uniformizeSpatialField(const std::vector<geometry_msgs::PoseStamped>& _posestamped_list);
    geographic_msgs::GeoPoseStamped poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped );
    geometry_msgs::PoseStamped geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped );
    mavros_msgs::Waypoint geoPoseStampedtoMavrosWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped );
    float getYaw(const geometry_msgs::Quaternion& _quat);

    // Atrributes used for the getters:
    geometry_msgs::PoseStamped  cur_pose_;
    sensor_msgs::NavSatFix      cur_geo_pose_;
    geometry_msgs::TwistStamped cur_vel_;
    float                       battery_percentage_;
    AutopilotType               autopilot_type_     = AutopilotType::UNKNOWN;
    AirframeType                airframe_type_      = AirframeType::UNKNOWN;
    bool                        armed_              = false;
    int                         active_waypoint_    = -1;   // seq nr of the currently active waypoint of the mission: waypoints[current_seq].is_current == True. -1 if not running a mission or disarmed.
    int                         robot_id_           = -1;

    // Ros Communications:
    ros::ServiceClient flight_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient get_param_client_;
    ros::ServiceClient set_param_client_;
    ros::ServiceClient push_mission_client_;
    ros::ServiceClient clear_mission_client_;
    ros::Subscriber mavros_cur_pose_sub_;
    ros::Subscriber mavros_cur_geo_pose_sub_;
    ros::Subscriber mavros_cur_vel_sub_;
    ros::Subscriber mavros_cur_state_sub_;
    ros::Subscriber mavros_cur_mission_sub_;
    ros::Subscriber drone_telemetry_sub_;

    mavros_msgs::State        mavros_state_;
    mavros_msgs::WaypointList mission_waypointlist_;      // The mission that will be pushed to the autopilot.

    std::string     pose_frame_id_;
    std::string     uav_home_frame_id_;
    Eigen::Vector3d local_start_pos_;
    geometry_msgs::Point32    map_origin_cartesian_;
    geographic_msgs::GeoPoint origin_geo_;

    tf2_ros::Buffer                                        tf_buffer_;
    tf2_ros::TransformListener                             tf_listener_;
    tf2_ros::StaticTransformBroadcaster *                  static_tf_broadcaster_;
    std::map<std::string, geometry_msgs::TransformStamped> cached_transforms_;

    std::string node_name_space_ = "";

    std::thread spin_thread_;       // Ros spinning threads for running callbacks

    bool mavros_has_geo_pose_   = false;
    bool uav_has_empty_mission_ = true;
    bool low_battery_warning_;
    bool id_is_unique_;
};

}	// namespace grvc

#endif // MISSION_LIB_H