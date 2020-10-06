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
#include <fixed_wing_lib/MissionElement.h>
#include <fixed_wing_lib/ParamFloat.h>
#include <fixed_wing_lib/State.h>
#include <fixed_wing_lib/SetHome.h>
#include <fixed_wing_lib/SetMission.h>

namespace grvc { namespace fw_ns {

class FixedWing {

public:

    FixedWing();
    ~FixedWing();

    // Wrap a function to make it thread-safe
    template <typename Callable, typename ... Args>
    bool threadSafeCall(Callable&& _fn, Args&& ... _args) {
        // Only one thread can lock
        if (running_mutex_.try_lock()) {
            running_task_ = true;  // set running after locking
            std::bind(_fn, this, std::forward<Args>(_args)...)();
            running_mutex_.unlock();
            running_task_ = false;  // reset it after unlocking
            return true;  // Call succeeded
        } else {
            return false;  // Call failed
        }
    }

    // Library is initialized and ready to run tasks?
    bool isReady() const;
    // Is it idle?
    bool isIdle() { return !running_task_; }

    // Latest pose estimation of the robot
    geometry_msgs::PoseStamped pose();
    // Latest velocity estimation of the robot
    geometry_msgs::TwistStamped velocity() const { return cur_vel_; }

    // Set home position
    bool setHome(bool _set_z);

    // Execute specified mission
    // \param waypoint set indicates the waypoint groups with its parameters
    bool setMission(const std::vector<fixed_wing_lib::MissionElement>& _waypoint_element_list);

    // Current robot state
    fixed_wing_lib::State state() {
        fixed_wing_lib::State output;
        output.state = this->state_;
        return output;
    };

    // Current waypoint of the list that define de mission
    std_msgs::Int8 missionState() {
        std_msgs::Int8 output;
        output.data = this->mission_state_;
        return output;
    };

    // Cancel execution of the current task
    void abort(bool _freeze = true);

private:
    void missionThreadLoop();
    void getAutopilotInformation();
    void initHomeFrame();
    void setFlightMode(const std::string& _flight_mode);
    double updateParam(const std::string& _param_id);
    fixed_wing_lib::State guessState();

    // FW specifics
    void arm(const bool& _arm);
    void setParam(const std::string& _param_id,const int& _param_value);
    bool pushMission(const mavros_msgs::WaypointList& _wp_list);
    void clearMission();
    void addTakeOffWp(mavros_msgs::WaypointList& _wp_list, const fixed_wing_lib::MissionElement& _waypoint_element, const int& _wp_set_index);
    void addPassWpList(mavros_msgs::WaypointList& _wp_list, const fixed_wing_lib::MissionElement& _waypoint_element, const int& _wp_set_index);
    void addLoiterWpList(mavros_msgs::WaypointList& _wp_list, const fixed_wing_lib::MissionElement& _waypoint_element, const int& _wp_set_index);
    void addLandWpList(mavros_msgs::WaypointList& _wp_list, const fixed_wing_lib::MissionElement& _waypoint_element, const int& _wp_set_index);
    void addSpeedWpList(mavros_msgs::WaypointList& _wp_list, const fixed_wing_lib::MissionElement& _waypoint_element, const int& _wp_set_index);
    std::vector<geographic_msgs::GeoPoseStamped> uniformizeSpatialField( const fixed_wing_lib::MissionElement& _waypoint_element);
    geographic_msgs::GeoPoseStamped poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped );
    geometry_msgs::PoseStamped geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped );
    mavros_msgs::Waypoint geoPoseStampedtoGlobalWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped );
    float getMissionYaw(const geometry_msgs::Quaternion& _quat);
    void checkMissionParams(const std::map<std::string, float>& _existing_params_map, const std::vector<std::string>& _required_params, const int& _wp_set_index);
    void initMission();
    void setMissionFunction(const std::vector<fixed_wing_lib::MissionElement>& _waypoint_element_list); // Function to execute a mission of a sequence of waypoints inside a safe thread call

    // WaypointList path_;
    geometry_msgs::PoseStamped  cur_pose_;
    sensor_msgs::NavSatFix      cur_geo_pose_;
    geometry_msgs::TwistStamped cur_vel_;
    mavros_msgs::State          mavros_state_;
    mavros_msgs::ExtendedState  mavros_extended_state_;

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
    ros::Publisher mission_state_pub_;
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

    std::thread mission_thread_;
    double mission_thread_frequency_;

    bool calling_takeoff_ = false;
    bool calling_land_ = false;

    std::atomic<uint8_t> state_ = {fixed_wing_lib::State::UNINITIALIZED};

    int mission_state_ = 0;     // seq nr of the currently active waypoint of the mission: waypoints[current_seq].is_current == True.

    // Abort flag
    // If you want your task to be abortable, check its value periodically
    std::atomic<bool> abort_ = {false};

    // Freeze flag
    // When aborting a task, freezes the platform if it is true
    std::atomic<bool> freeze_ = {false};

    // Simplest state-machine model: idle/running
    // Implemented via mutex-locking
    std::mutex running_mutex_;
    std::atomic<bool> running_task_ = {false};

    // Ros spinning thread
    std::thread spin_thread_;

    std::thread server_thread_;
};

}}	// namespace grvc::fw_ns

#endif // FIXED_WING_LIB_H