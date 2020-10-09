//----------------------------------------------------------------------------------------------------------------------
// Fixed wing lib
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
#ifndef FIXED_WING_LIB_H
#define FIXED_WING_LIB_H

#include <ros/ros.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <Eigen/Core>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/Int8.h>
#include <fixed_wing_lib/State.h>
#include <fixed_wing_lib/SetHome.h>

namespace grvc { namespace fw_ns {

class FixedWing {

public:

    FixedWing();
    ~FixedWing();

    // Latest pose estimation of the robot
    geometry_msgs::PoseStamped pose();
    sensor_msgs::NavSatFix geoPose() const { return cur_geo_pose_; }
    // Latest velocity estimation of the robot
    geometry_msgs::TwistStamped velocity() const { return cur_vel_; }

    // Current uav state
    fixed_wing_lib::State state() const {
        fixed_wing_lib::State output;
        output.state = this->state_;
        return output;
    };

    // Current waypoint of the list that define de mission
    std_msgs::Int8 activeWaypoint() const {
        std_msgs::Int8 output;
        output.data = this->active_waypoint_;
        return output;
    };

    // Set home position
    bool setHome(bool _set_z);

    bool pushMission();
    void clearMission();
    void printMission();
    void addTakeOffWp(const geometry_msgs::PoseStamped& _takeoff_pose, float _minimum_pitch, float _aux_distance=-1, float _aux_height=-1, float _yaw_angle=-1);
    void addPassWpList(const std::vector<geometry_msgs::PoseStamped>& _pass_poses, float _acceptance_radius, float _orbit_distance, float _speed=-1);
    void addLoiterWpList(const std::vector<geometry_msgs::PoseStamped>& _loiter_poses, float _radius, float _forward_moving=-1, float _turns=-1, float _time=-1, float _heading=-1, float _speed=-1);
    void addLandWpList(const geometry_msgs::PoseStamped& _land_pose, float _loit_heading, float _loit_radius, float _loit_forward_moving, float _abort_alt, float _precision_mode, float _aux_distance=-1, float _aux_height=-1, float _aux_angle=-1);
    void addSpeedWpList(float _speed);

private:
    // Library is initialized and ready to send missions?
    bool isReady() const;

    void stateThreadLoop();
    void getAutopilotInformation();
    void initHomeFrame();
    void setFlightMode(const std::string& _flight_mode);
    double updateParam(const std::string& _param_id);
    fixed_wing_lib::State guessState();

    // FW specifics
    void arm(bool _arm);
    void setParam(const std::string& _param_id,int _param_value);
    std::vector<geographic_msgs::GeoPoseStamped> uniformizeSpatialField(const std::vector<geometry_msgs::PoseStamped>& _posestamped_list);
    geographic_msgs::GeoPoseStamped poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped );
    geometry_msgs::PoseStamped geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped );
    mavros_msgs::Waypoint geoPoseStampedtoGlobalWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped );
    float getMissionYaw(const geometry_msgs::Quaternion& _quat);
    void initMission();

    // WaypointList path_;
    geometry_msgs::PoseStamped  cur_pose_;
    sensor_msgs::NavSatFix      cur_geo_pose_;
    geometry_msgs::TwistStamped cur_vel_;
    mavros_msgs::State          mavros_state_;
    mavros_msgs::ExtendedState  mavros_extended_state_;
    mavros_msgs::WaypointList   mission_waypointlist_;      // The mission that will be pushed to the autopilot.

    // Mission
    mavros_msgs::WaypointList mavros_cur_mission_;
    geographic_msgs::GeoPoint origin_geo_;
    std::vector<int> takeoff_wps_on_mission_;
    std::vector<int> land_wps_on_mission_;

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
    ros::ServiceServer set_mission_service_;
    ros::ServiceServer set_home_service_;
    ros::Subscriber mavros_cur_pose_sub_;
    ros::Subscriber mavros_cur_geo_pose_sub_;
    ros::Subscriber mavros_cur_vel_sub_;
    ros::Subscriber mavros_cur_state_sub_;
    ros::Subscriber mavros_cur_extended_state_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher pose_geo_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher state_pub_;
    ros::Publisher active_waypoint_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber mavros_cur_mission_sub_;

    int robot_id_;
    bool id_is_unique_;
    enum struct AutopilotType {PX4, APM, UNKNOWN};
    AutopilotType autopilot_type_ = AutopilotType::UNKNOWN;
    std::string pose_frame_id_;
    std::string uav_home_frame_id_;
    std::string uav_frame_id_;
    tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
    std::map<std::string, double> mavros_params_;
    Eigen::Vector3d local_start_pos_;

    std::thread state_thread_;
    double state_thread_frequency_;

    bool calling_takeoff_ = false;
    bool calling_land_ = false;

    std::atomic<uint8_t> state_ = {fixed_wing_lib::State::UNINITIALIZED};

    int active_waypoint_ = 0;     // seq nr of the currently active waypoint of the mission: waypoints[current_seq].is_current == True.

    // Ros spinning thread
    std::thread spin_thread_;

    std::thread server_thread_;
};

}}	// namespace grvc::fw_ns

#endif // FIXED_WING_LIB_H