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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace grvc { namespace mission_ns {

class Mission {

public:

    Mission();
    ~Mission();

    // Latest pose estimation of the robot
    geometry_msgs::PoseStamped pose();
    sensor_msgs::NavSatFix geoPose() const { return this->cur_geo_pose_; }
    // Latest velocity estimation of the robot
    geometry_msgs::TwistStamped velocity() const { return this->cur_vel_; }

    // Current waypoint of the list that define de mission
    int activeWaypoint() const { return this->active_waypoint_; };

    // Set home position
    bool setHome(bool _set_z);

    void addTakeOffWp(const geometry_msgs::PoseStamped& _takeoff_pose, float _minimum_pitch, float _yaw_angle=-1);
    void addPassWpList(const std::vector<geometry_msgs::PoseStamped>& _pass_poses, float _acceptance_radius, float _orbit_distance, float _speed=-1);
    void addLoiterWpList(const std::vector<geometry_msgs::PoseStamped>& _loiter_poses, float _radius, float _forward_moving=-1, float _turns=-1, float _time=-1, float _heading=-1, float _speed=-1);
    void addLandWpList(const std::vector<geometry_msgs::PoseStamped>& _land_poses, float _loit_heading, float _loit_radius, float _loit_forward_moving, float _abort_alt, float _precision_mode);
    void addSpeedWp(float _speed);
    void print() const;
    void clear();
    bool push();
    bool pushClear();
    void start();

private:
    // Library is initialized and ready to send missions?
    bool isReady() const;

    void getAutopilotInformation();
    void initHomeFrame();
    void setFlightMode(const std::string& _flight_mode);
    void setParam(const std::string& _param_id,int _param_value);
    double updateParam(const std::string& _param_id);
    std::map<std::string, double> mavros_params_;

    void arm(bool _arm);
    std::vector<geographic_msgs::GeoPoseStamped> uniformizeSpatialField(const std::vector<geometry_msgs::PoseStamped>& _posestamped_list);
    geographic_msgs::GeoPoseStamped poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped );
    geometry_msgs::PoseStamped geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped );
    mavros_msgs::Waypoint geoPoseStampedtoGlobalWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped );
    float getYaw(const geometry_msgs::Quaternion& _quat);

    geometry_msgs::PoseStamped  cur_pose_;
    sensor_msgs::NavSatFix      cur_geo_pose_;
    geometry_msgs::TwistStamped cur_vel_;
    mavros_msgs::State          mavros_state_;

    // Mission
    geographic_msgs::GeoPoint origin_geo_;
    mavros_msgs::WaypointList mission_waypointlist_;      // The mission that will be pushed to the autopilot.

    // Control
    bool mavros_has_pose_ = false;
    bool mavros_has_geo_pose_ = false;

    // Ros Communication
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
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int robot_id_;
    bool id_is_unique_;

    enum struct AutopilotType {PX4, APM, UNKNOWN};
    AutopilotType autopilot_type_ = AutopilotType::UNKNOWN;

    enum struct AirframeType {FIXED_WING, MULTICOPTER, VTOL, OTHER, UNKNOWN};
    AirframeType airframe_type_ = AirframeType::UNKNOWN;

    std::string pose_frame_id_;
    std::string uav_home_frame_id_;
    std::string uav_frame_id_;
    Eigen::Vector3d local_start_pos_;
    tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    std::map<std::string, geometry_msgs::TransformStamped> cached_transforms_;

    int active_waypoint_ = -1;      // seq nr of the currently active waypoint of the mission: waypoints[current_seq].is_current == True.

    std::thread spin_thread_;       // Ros spinning threads for running callbacks
};

}}	// namespace grvc::mission_ns

#endif // MISSION_LIB_H