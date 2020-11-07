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
#include <geographic_to_cartesian.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <mavros/utils.h>
#include <string>
#include <chrono>
#include <Eigen/Eigen>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>

#define FIRMWARE_VERSION_TYPE_DEV 0 /* development release | */
#define FIRMWARE_VERSION_TYPE_ALPHA 64 /* alpha release | */
#define FIRMWARE_VERSION_TYPE_BETA 128 /* beta release | */
#define FIRMWARE_VERSION_TYPE_RC 192 /* release candidate | */
#define FIRMWARE_VERSION_TYPE_OFFICIAL 255 /* official stable release | */
#define FIRMWARE_VERSION_TYPE_ENUM_END 256 /*  | */

namespace grvc { namespace mission_ns {

Mission::Mission(int _uav_id)
    : tf_listener_(tf_buffer_)
{
    robot_id_ = _uav_id;
    Mission();
}


Mission::Mission(int _uav_id, std::string _pose_frame_id)
    : tf_listener_(tf_buffer_)
{
    robot_id_ = _uav_id;
    pose_frame_id_ = _pose_frame_id;
    Mission();
}


Mission::Mission()
    : tf_listener_(tf_buffer_)
{
    // Error if ROS is not initialized
    if (!ros::isInitialized()) {
        // Init ros node
        ROS_ERROR("Mission_lib needs ROS to be initialized. Initialize ROS before creating a Mission object.");
        exit(EXIT_FAILURE);
    }

    // Parse arguments
    ros::NodeHandle pnh("~");
    if (robot_id_==-1)      pnh.param<int>("uav_id", robot_id_, 1);
    if (pose_frame_id_=="") pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");

    // Assure id uniqueness
    id_is_unique_ = true;
    std::vector<int> uav_ids;
    if (ros::param::has("/uav_ids")) {
        ros::param::get("/uav_ids", uav_ids);
        for (auto id: uav_ids) {
            if (id == robot_id_) {
                id_is_unique_ = false;
            }
        }
        if (!id_is_unique_) {
            ROS_ERROR("Another UAV with id [%d] is already running!", robot_id_);
            throw std::runtime_error("Id is not unique, already found in /uav_ids");
        }
    }
    if (id_is_unique_) {
        uav_ids.push_back(robot_id_);
    }
    ros::param::set("/uav_ids", uav_ids);

    // Init variables
    cur_pose_.pose.orientation.x = 0;
    cur_pose_.pose.orientation.y = 0;
    cur_pose_.pose.orientation.z = 0;
    cur_pose_.pose.orientation.w = 1;

    ROS_INFO("Mission_lib constructor with robot id [%d]", robot_id_);

    // Init ros communications
    ros::NodeHandle nh;
    std::string mavros_ns = "mavros";
    std::string set_mode_srv = mavros_ns + "/set_mode";
    std::string arming_srv = mavros_ns + "/cmd/arming";
    std::string get_param_srv = mavros_ns + "/param/get";
    std::string set_param_srv = mavros_ns + "/param/set";
    std::string push_mission_srv = mavros_ns + "/mission/push";
    std::string clear_mission_srv = mavros_ns + "/mission/clear";
    std::string pose_topic = mavros_ns + "/local_position/pose";
    std::string geo_pose_topic = mavros_ns + "/global_position/global";
#ifdef MAVROS_VERSION_BELOW_0_29_0
    std::string vel_topic_local = mavros_ns + "/local_position/velocity";
#else
    std::string vel_topic_local = mavros_ns + "/local_position/velocity_local";
#endif
    std::string state_topic = mavros_ns + "/state";
    std::string waypoints_mission_topic = mavros_ns + "/mission/waypoints";

    flight_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str());
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(arming_srv.c_str());
    // Client to push missions to mavros
    push_mission_client_ = nh.serviceClient<mavros_msgs::WaypointPush>(push_mission_srv.c_str());
    // Client to clear the mission on mavros
    clear_mission_client_ = nh.serviceClient<mavros_msgs::WaypointClear>(clear_mission_srv.c_str());
    // Client to set parameters from mavros
    set_param_client_ = nh.serviceClient<mavros_msgs::ParamSet>(set_param_srv.c_str());

    mavros_cur_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic.c_str(), 1, \
        [this](const geometry_msgs::PoseStamped::ConstPtr& _msg) {
            this->cur_pose_ = *_msg;
            this->mavros_has_pose_ = true;
    });
    mavros_cur_vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic_local.c_str(), 1, \
        [this](const geometry_msgs::TwistStamped::ConstPtr& _msg) {
            this->cur_vel_ = *_msg;
            this->cur_vel_.header.frame_id = this->uav_home_frame_id_;
    });
    mavros_cur_geo_pose_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(geo_pose_topic.c_str(), 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->cur_geo_pose_ = *_msg;
            if (!this->mavros_has_geo_pose_) {
                if (_msg->position_covariance[0] < 1.2 && _msg->position_covariance[0] > 0 && _msg->header.seq > 100) {
                    this->mavros_has_geo_pose_ = true;
                    // ROS_INFO("Has Geo Pose! %f",_msg->position_covariance[0]);
                }
            }
    });
    mavros_cur_state_sub_ = nh.subscribe<mavros_msgs::State>(state_topic.c_str(), 1, \
        [this](const mavros_msgs::State::ConstPtr& _msg) {
            this->mavros_state_ = *_msg;
            if (uav_has_empty_mission_ || !mavros_state_.armed) {
                this->active_waypoint_ = -1;
            }
    });

    mavros_cur_mission_sub_ = nh.subscribe<mavros_msgs::WaypointList>(waypoints_mission_topic.c_str(), 1, \
        [this](const mavros_msgs::WaypointList::ConstPtr& _msg) {
            uav_has_empty_mission_ = _msg->waypoints.size()==0 ? true : false;
            if (uav_has_empty_mission_ || !mavros_state_.armed) {
                this->active_waypoint_ = -1;
            } else {
                this->active_waypoint_ = _msg->current_seq;
            }
    });

    // Make communications spin!
    spin_thread_ = std::thread([this]() {
        ros::MultiThreadedSpinner spinner(2); // Use 2 threads
        spinner.spin();
    });

    // Wait until mavros is connected
    while (!mavros_state_.connected && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    getAutopilotInformation();

    // Error if the autpilot is not PX4, as right now is the only one supported by this library.
    if (autopilot_type_!=AutopilotType::PX4) {
        ROS_ERROR("Mission_lib only works for PX4 Autopilot. Aborting Mission constructor.");
        exit(EXIT_FAILURE);
    }

    // Error if the airframe is not a fixed wing, multicopter or VTOL, as right now they are the ones supported by this library.
    if (airframe_type_==AirframeType::OTHER) {
        ROS_ERROR("Mission_lib only works for fixed wing, multicopter or VTOL airframe. Aborting Mission constructor.");
        exit(EXIT_FAILURE);
    }

    // TODO: Check this and solve frames issue
    initHomeFrame();

    // Client to get parameters from mavros and required default values
    get_param_client_ = nh.serviceClient<mavros_msgs::ParamGet>(get_param_srv.c_str());
    // Updating here is non-sense as service seems to be slow in waking up

    // The following param need to be set in the px4cmd, setting them from here is not possible for MAVROS (in QGroundStation is possible):
    // setParam("NAV_DLL_ACT",0);   // To switch mode
    // setParam("MIS_DIST_1WP",0);
    // setParam("MIS_DIST_WPS",0);  // Minimum distance between wps. default 900m
}


Mission::~Mission() {
    if (spin_thread_.joinable()) { spin_thread_.join(); }

    if (id_is_unique_) {
        // Remove id from /uav_ids
        std::vector<int> uav_ids;
        ros::param::get("/uav_ids", uav_ids);
        std::vector<int> new_uav_ids;
        for (auto id: uav_ids) {
            if (id != robot_id_) {
                new_uav_ids.push_back(id);
            }
        }
        ros::param::set("/uav_ids", new_uav_ids);
    }
}


void Mission::setFlightMode(const std::string& _flight_mode) {
    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = _flight_mode;
    // Set mode: unabortable?
    while (mavros_state_.mode != _flight_mode && ros::ok()) {
        if (!flight_mode_client_.call(flight_mode_service)) {
            ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.success ? "true" : "false");
#else
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", _flight_mode.c_str(), mavros_state_.mode.c_str());
    }
}


void Mission::arm(bool _arm) {
    mavros_msgs::CommandBool arm_service;
    arm_service.request.value = _arm;

    // Set mode: unabortable?
    while (mavros_state_.armed != _arm && ros::ok()) {
        if (!arming_client_.call(arm_service)) {
            ROS_ERROR("Error in [%s] service calling!", _arm ? "arming" : "disarming");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set [%s] response.success = %s", _arm ? "armed" : "disarmed", \
            arm_service.response.success ? "true" : "false");
#else
        // ROS_INFO("Set [%s] response.success = %s", _arm ? "armed" : "disarmed", \
        //     arm_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("  Trying to [%s]; mavros_state_.armed = [%s]", _arm ? "arm" : "disarm", mavros_state_.armed ? "true" : "false");
    }
}


bool Mission::setHome(bool _set_z) {
    // The following check was removed when this library became unaware of the state, so now PX4 will be the one complaining when unable to setHome.
    // // Check required state
    // if ((this->state().state != fixed_wing_lib::State::LANDED_DISARMED) && (this->state().state != fixed_wing_lib::State::LANDED_ARMED)) {
    //     ROS_ERROR("Unable to setHome: not LANDED_*!");
    //     return false;
    // }

    double z_offset = _set_z ? cur_pose_.pose.position.z : 0.0;
    local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
        cur_pose_.pose.position.y, z_offset);

    return true;
}


bool Mission::isReady() const {
    if (ros::param::has("~map_origin_geo")) {
        return mavros_has_geo_pose_;
    } else {
        return mavros_has_pose_ && (fabs(this->cur_pose_.pose.position.y) > 1e-8);  // Means the filter has converged!
    }
}


geometry_msgs::PoseStamped Mission::pose() {
    geometry_msgs::PoseStamped out;

    out.pose.position.x = cur_pose_.pose.position.x + local_start_pos_[0];
    out.pose.position.y = cur_pose_.pose.position.y + local_start_pos_[1];
    out.pose.position.z = cur_pose_.pose.position.z + local_start_pos_[2];
    out.pose.orientation = cur_pose_.pose.orientation;

    if (pose_frame_id_ == "") {
        // Default: local pose
        out.header.frame_id = uav_home_frame_id_;
    } else {
        // Publish pose in different frame
        geometry_msgs::PoseStamped aux = out;
        geometry_msgs::TransformStamped transformToPoseFrame;
        std::string pose_frame_id_map = "inv_" + pose_frame_id_;

        if ( cached_transforms_.find(pose_frame_id_map) == cached_transforms_.end() ) {
            // inv_pose_frame_id_ not found in cached_transforms_
            try {
                transformToPoseFrame = tf_buffer_.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
                cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
            } catch (tf2::TransformException &ex) {
                ROS_WARN("In pose: %s. Returning non transformed pose.", ex.what());
                return out;
            }
        } else {
            // found in cache
            transformToPoseFrame = cached_transforms_[pose_frame_id_map];
        }

        tf2::doTransform(aux, out, transformToPoseFrame);
        out.header.frame_id = pose_frame_id_;
    }

    out.header.stamp = cur_pose_.header.stamp;
    return out;
}


void Mission::initHomeFrame() {

    local_start_pos_ << 0.0, 0.0, 0.0;

    // Get frame prefix from namespace
    std::string ns = ros::this_node::getNamespace();
    uav_frame_id_ = ns + "/base_link";
    uav_home_frame_id_ = ns + "/odom";
    while (uav_frame_id_[0]=='/') {
        uav_frame_id_.erase(0,1);
    }
    while (uav_home_frame_id_[0]=='/') {
        uav_home_frame_id_.erase(0,1);
    }
    std::string parent_frame;
    ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");

    std::vector<double> home_pose(3, 0.0);
    if (ros::param::has("~home_pose")) {
        ros::param::get("~home_pose",home_pose);
    } else if (ros::param::has("~map_origin_geo")) {
        ROS_WARN("Be careful, you should only use this mode with RTK GPS!");
        while (!this->mavros_has_geo_pose_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        std::vector<double> map_origin_geo(3, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        geographic_msgs::GeoPoint origin_geo, actual_coordinate_geo;
        origin_geo.latitude = map_origin_geo[0];
        origin_geo.longitude = map_origin_geo[1];
        origin_geo.altitude = 0; //map_origin_geo[2];
        actual_coordinate_geo.latitude = cur_geo_pose_.latitude;
        actual_coordinate_geo.longitude = cur_geo_pose_.longitude;
        actual_coordinate_geo.altitude = 0; //cur_geo_pose_.altitude;
        if(map_origin_geo[0]==0 && map_origin_geo[1]==0) {
            ROS_WARN("Map origin is set to 0. Define map_origin_geo param by a vector in format [lat,lon,alt].");
        }
        geometry_msgs::Point32 map_origin_cartesian = geographic_to_cartesian (actual_coordinate_geo, origin_geo);

        home_pose[0] = map_origin_cartesian.x;
        home_pose[1] = map_origin_cartesian.y;
        home_pose[2] = map_origin_cartesian.z;
    } else {
        ROS_WARN("No home pose or map origin was defined. Home frame will be equal to map.");
    }

    if (ros::param::has("~map_origin_geo")) {

        std::vector<double> map_origin_geo(0.0, 0.0);
        ros::param::get("~map_origin_geo",map_origin_geo);
        origin_geo_.latitude = map_origin_geo[0];
        origin_geo_.longitude = map_origin_geo[1];
        origin_geo_.altitude = 0;                           //map_origin_geo[2];

    }

    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent_frame;
    static_transformStamped.child_frame_id = uav_home_frame_id_;
    static_transformStamped.transform.translation.x = home_pose[0];
    static_transformStamped.transform.translation.y = home_pose[1];
    static_transformStamped.transform.translation.z = home_pose[2];
    static_transformStamped.transform.rotation.x = 0;
    static_transformStamped.transform.rotation.y = 0;
    static_transformStamped.transform.rotation.z = 0;
    static_transformStamped.transform.rotation.w = 1;

    if(parent_frame != "map" && parent_frame != "") {
        geometry_msgs::TransformStamped transform_to_map;
        try {
            transform_to_map = tf_buffer_.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
            static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("In initHomeFrame: %s. Publishing static TF in ENU.", ex.what());
        }
    }

    static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
    static_tf_broadcaster_->sendTransform(static_transformStamped);     // Needed?
}


double Mission::updateParam(const std::string& _param_id) {
    mavros_msgs::ParamGet get_param_service;
    get_param_service.request.param_id = _param_id;
    if (get_param_client_.call(get_param_service) && get_param_service.response.success) {
        mavros_params_[_param_id] = get_param_service.response.value.integer? 
            get_param_service.response.value.integer : get_param_service.response.value.real;
        ROS_DEBUG("Parameter [%s] value is [%f]", get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else if (mavros_params_.count(_param_id)) {
        ROS_WARN("Error in get param [%s] service calling, leaving current value [%f]", 
            get_param_service.request.param_id.c_str(), mavros_params_[_param_id]);
    } else {
        mavros_params_[_param_id] = 0.0;
        ROS_ERROR("Error in get param [%s] service calling, initializing it to zero", 
            get_param_service.request.param_id.c_str());
    }
    return mavros_params_[_param_id];
}


void Mission::setParam(const std::string& _param_id, int _param_value) {
    mavros_msgs::ParamSet set_param_service;
    set_param_service.request.param_id = _param_id;
    set_param_service.request.value.integer = _param_value;     // FIX FOR FLOAT
    set_param_service.request.value.real = 0;

    while (updateParam(_param_id) != _param_value && ros::ok()) {
        if (!set_param_client_.call(set_param_service)) {
            ROS_ERROR("Error in set param [%s] service calling!", _param_id.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
            set_param_service.response.success ? "true" : "false");
#else
        // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
        //     set_param_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] param to [%10d]", _param_id.c_str(), _param_value);
    }
}


void Mission::getAutopilotInformation() {
    // Call vehicle information service
    ros::NodeHandle nh;
    ros::ServiceClient vehicle_information_cl = nh.serviceClient<mavros_msgs::VehicleInfoGet>("mavros/vehicle_info_get");
    ros::service::waitForService("mavros/vehicle_info_get");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    mavros_msgs::VehicleInfoGet vehicle_info_srv;
    if (!vehicle_information_cl.call(vehicle_info_srv)) {
        ROS_ERROR("Failed to get vehicle information: service call failed");
        exit(0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!vehicle_info_srv.response.success) {
        ROS_ERROR("Failed to get vehicle information");
        exit(0);
    }
    // Autopilot type
    switch (vehicle_info_srv.response.vehicles[0].autopilot) {
        case 3:
            autopilot_type_ = AutopilotType::APM;
            break;
        case 12:
            autopilot_type_ = AutopilotType::PX4;
            break;
        default:
            ROS_ERROR("Mission [%d]: Wrong autopilot type: %s", robot_id_, mavros::utils::to_string((mavlink::minimal::MAV_AUTOPILOT) vehicle_info_srv.response.vehicles[0].autopilot).c_str());
            exit(0);
    }

    // Autopilot version
    int major_version = ( ((int) vehicle_info_srv.response.vehicles[0].flight_sw_version) >> (8*3)) & 0xFF;
    int minor_version = (vehicle_info_srv.response.vehicles[0].flight_sw_version >> (8*2)) & 0xFF;
    int patch_version = (vehicle_info_srv.response.vehicles[0].flight_sw_version >> (8*1)) & 0xFF;
    int version_type_int = (vehicle_info_srv.response.vehicles[0].flight_sw_version >> (8*0)) & 0xFF;
    std::string version_type;
    switch (version_type_int) {
        case FIRMWARE_VERSION_TYPE_DEV:
            version_type = "dev";
            break;
        case FIRMWARE_VERSION_TYPE_ALPHA:
            version_type = "alpha";
            break;
        case FIRMWARE_VERSION_TYPE_BETA:
            version_type = "beta";
            break;
        case FIRMWARE_VERSION_TYPE_RC:
            version_type = "rc";
            break;
        case FIRMWARE_VERSION_TYPE_OFFICIAL:
        default:
            version_type = "";
    }

    // Airframe type
    switch (vehicle_info_srv.response.vehicles[0].type) {
        case 1:     // Fixed wing aircraft
            airframe_type_ = AirframeType::FIXED_WING;
            break;
        case 2:     // Quadrotor
        case 3:     // Coaxial helicopter
        case 4:     // Normal helicopter with tail rotor
        case 13:    // Hexarotor
        case 14:    // Octorotor
        case 15:    // Tricopter
            airframe_type_ = AirframeType::MULTICOPTER;
            break;
        case 19:    // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
        case 20:    // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
        case 21:    // Tiltrotor VTOL
        case 22:    // VTOL reserved 2
        case 23:    // VTOL reserved 3
        case 24:    // VTOL reserved 4
        case 25:    // VTOL reserved 5
            airframe_type_ = AirframeType::VTOL;
            break;
        case 0:     // Generic micro air vehicle
        case 5:     // Ground installation
        case 6:     // Operator control unit / ground control station
        case 7:     // Airship, controlled
        case 8:     // Free balloon, uncontrolled
        case 9:     // Rocket
        case 10:    // Ground rover
        case 11:    // Surface vessel, boat, ship
        case 12:    // Submarine
        case 16:    // Flapping wing
        case 17:    // Kite
        case 18:    // Onboard companion controller
        case 26:    // Onboard gimbal
        case 27:    // Onboard ADSB peripheral
            airframe_type_ = AirframeType::OTHER;
            break;
        default:
            ROS_ERROR("Mission [%d]: Wrong airframe type: %s", robot_id_, mavros::utils::to_string((mavlink::minimal::MAV_TYPE) vehicle_info_srv.response.vehicles[0].type).c_str());
            exit(0);
    }

    std::string autopilot_version = std::to_string(major_version) + "." + std::to_string(minor_version) + "." + std::to_string(patch_version) + version_type;

    // Autopilot string
    ROS_INFO("Mission [%d]: Connected to %s version %s. Type: %s.", robot_id_,
    mavros::utils::to_string((mavlink::minimal::MAV_AUTOPILOT) vehicle_info_srv.response.vehicles[0].autopilot).c_str(),
    autopilot_version.c_str(), mavros::utils::to_string((mavlink::minimal::MAV_TYPE) vehicle_info_srv.response.vehicles[0].type).c_str());
}


bool Mission::push() {
    mavros_msgs::WaypointPush push_waypoint_service;
    push_waypoint_service.request.start_index = 0;
    push_waypoint_service.request.waypoints = mission_waypointlist_.waypoints;
    ROS_INFO("Trying to push mission");

    if (!push_mission_client_.call(push_waypoint_service)) {
        ROS_ERROR("Error in push mission service calling!");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
    ROS_INFO("Push mission response.success = %s", set_param_service.response.success ? "true" : "false");
#else
    // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
    //     set_param_service.response.mode_sent ? "true" : "false");
#endif

    ROS_INFO("Push mission ended.");

    return push_waypoint_service.response.success;
}


bool Mission::pushClear() {
    // The following check was removed when this library became unaware of the state, so now PX4 will be the one complaining when unable to puxhClear.
    // // Check required state
    // if ((this->state().state != fixed_wing_lib::State::LANDED_DISARMED) && (this->state().state != fixed_wing_lib::State::LANDED_ARMED)) {
    //     ROS_ERROR("Unable to pushClear: not LANDED_*!");
    // }

    mavros_msgs::WaypointClear clear_mission_service;

    if (!clear_mission_client_.call(clear_mission_service)) {
        ROS_ERROR("Error in clear mission service calling!");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
    ROS_INFO("Clear mission response.success = %s", clear_mission_service.response.success ? "true" : "false");
#else
    // ROS_INFO("Set param [%s] response.success = %s", _param_id.c_str(), \
    //     set_param_service.response.mode_sent ? "true" : "false");
#endif
    ROS_INFO("Trying to clear mission");

    return true;
}


void Mission::start() {
    if (uav_has_empty_mission_) {
        ROS_ERROR("Mission start() called but the UAV doesn't have a mission. Ignoring the start() call.");
    } else if (active_waypoint_ != -1) {
        ROS_ERROR("Mission start() called but the UAV is already doing a mission. Ignoring the start() call.");
    } else {
        setFlightMode("AUTO.MISSION");
        arm(false); 
        arm(true);
    }
}


void Mission::clear() {
    mission_waypointlist_.waypoints.clear();
}


void Mission::addTakeOffWp(const geometry_msgs::PoseStamped& _takeoff_pose, float _minimum_pitch) {

    mavros_msgs::Waypoint wp;

    float yaw_angle;

    std::vector<geometry_msgs::PoseStamped> takeoff_pose_vector;
    takeoff_pose_vector.push_back(_takeoff_pose);

    std::vector<geographic_msgs::GeoPoseStamped> usf;   // Stands for Uniformized Spatial Field
    usf = uniformizeSpatialField(takeoff_pose_vector);

    if (usf.size() != 1) { ROS_ERROR("Error in [%d]-th waypoint set, posestamped list lenght is not 1!", (int) mission_waypointlist_.waypoints.size()); } //TODO(JoseAndres): Update errors

    yaw_angle = getYaw(usf[0].pose.orientation);

    wp = geoPoseStampedtoGlobalWaypoint(usf[0]);

    wp.frame = 3;           // FRAME_GLOBAL_REL_ALT

    if (airframe_type_==AirframeType::VTOL) {
        wp.command = 84;    // MAV_CMD_NAV_VTOL_TAKEOFF
    } else  {
        wp.command = 22;    // MAV_CMD_NAV_TAKEOFF
        wp.param1 = _minimum_pitch; // (if airspeed sensor present), desired pitch without sensor
    }

    wp.is_current = mission_waypointlist_.waypoints.size()==0? true : false;
    wp.autocontinue = true;
    wp.param4 = yaw_angle;      // (if magnetometer present), ignored without magnetometer. NaN for unchanged.

    mission_waypointlist_.waypoints.push_back(wp);
}


void Mission::addPassWpList(const std::vector<geometry_msgs::PoseStamped>& _pass_poses, float _speed, float _acceptance_radius, float _pass_radius) {

    std::vector<geographic_msgs::GeoPoseStamped> usf;
    usf = uniformizeSpatialField(_pass_poses);

    if (usf.size() == 0) { ROS_ERROR("Error in [%d]-th waypoint set, posestamped list is empty!", (int) mission_waypointlist_.waypoints.size()); }

    if (_speed!=-1) {
        addSpeedWp(_speed);
    }

    for ( auto & geoposestamped : usf ) {

        mavros_msgs::Waypoint wp = geoPoseStampedtoGlobalWaypoint(geoposestamped);
        wp.frame = 3;           // FRAME_GLOBAL_REL_ALT
        wp.command = 16;        // MAV_CMD_NAV_WAYPOINT
        wp.is_current = false;
        wp.autocontinue = true;
        wp.param2 = _acceptance_radius;                     // (if the sphere with this radius is hit, the waypoint counts as reached)
        wp.param3 = _pass_radius;                           // 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit,
                                                            // negative value for counter-clockwise orbit. Allows trajectory control.
        wp.param4 = getYaw(geoposestamped.pose.orientation); // Desired yaw angle at waypoint (rotary wing). NaN for unchanged.

        mission_waypointlist_.waypoints.push_back(wp);

    }
}


void Mission::addLoiterWpList(const std::vector<geometry_msgs::PoseStamped>& _loiter_poses, float _time, float _radius, float _speed, float _turns, float _forward_moving, float _heading) {

    std::vector<geographic_msgs::GeoPoseStamped> usf;
    usf = uniformizeSpatialField(_loiter_poses);

    if (usf.size() == 0) { ROS_ERROR("Error in [%d]-th waypoint set, posestamped list is empty!", (int) mission_waypointlist_.waypoints.size()); }

    if (_speed!=-1) {
        addSpeedWp(_speed);
    }

    for ( auto & geoposestamped : usf ) {

        mavros_msgs::Waypoint wp;
        wp = geoPoseStampedtoGlobalWaypoint( geoposestamped);
        wp.frame = 3;           // FRAME_GLOBAL_REL_ALT
        wp.is_current = false;
        wp.autocontinue = true;

        if (airframe_type_==AirframeType::MULTICOPTER) {
            if (_time==-1) {
                // LOITER_UNLIMITED

                wp.command = 17;                                     // MAV_CMD_NAV_LOITER_UNLIM
                wp.param4 = getYaw(geoposestamped.pose.orientation); // NaN for unchanged.

            } else {
                // LOITER_TIME

                wp.command = 19;                // MAV_CMD_NAV_LOITER_TIME
                wp.param1 = _time;              // Loiter time.

            }
        } else {
            if (_turns==-1 && _time==-1 && _heading==-1) {
                // LOITER_UNLIMITED

                wp.command = 17;                                     // MAV_CMD_NAV_LOITER_UNLIM
                wp.param3 = _radius;                                 // Radius around waypoint. If positive loiter clockwise, else counter-clockwise
                wp.param4 = getYaw(geoposestamped.pose.orientation); // NaN for unchanged.

            } else if (_turns!=-1 && _time==-1 && _heading==-1) {
                // LOITER_TURNS

                wp.command = 18;                // MAV_CMD_NAV_LOITER_TURNS
                wp.param1 = _turns;             // Number of turns.
                wp.param3 = _radius;            // Radius around waypoint. If positive loiter clockwise, else counter-clockwise (not multicopters)
                wp.param4 = _forward_moving;    // this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location.
                                                // Else, this is desired yaw angle. NaN for unchanged. (not multicopters)

            } else if (_turns==-1 && _time!=-1 && _heading==-1) {
                // LOITER_TIME

                wp.command = 19;                // MAV_CMD_NAV_LOITER_TIME
                wp.param1 = _time;              // Loiter time.
                wp.param2 = 0;                  // Leave loiter circle only once heading towards the next waypoint (0 = False)
                wp.param3 = _radius;            // Radius around waypoint. If positive loiter clockwise, else counter-clockwise. (not multicopters)
                wp.param4 = _forward_moving;    // this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location.
                                                // Else, this is desired yaw angle. NaN for unchanged. (not multicopters)

            } else if (_turns==-1 && _time==-1 && _heading!=-1) {
                // LOITER_HEIGHT

                wp.command = 31;                // MAV_CMD_NAV_LOITER_TO_ALT
                wp.param1 = _heading;           // Heading Required (0 = False)
                wp.param2 = _radius;            // If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter. (not multicopters)
                wp.param4 = _forward_moving;    // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location (not multicopters)

            }
        }

        mission_waypointlist_.waypoints.push_back(wp);
    }
}


void Mission::addLandWp(const geometry_msgs::PoseStamped& _land_pose, float _abort_alt, float _precision_mode) {

    if (airframe_type_==AirframeType::FIXED_WING) {
        ROS_ERROR("Error in [%d]-th waypoint, _loiter_to_alt_start_landing_pose missing in Mission::addLandWp for airframe type fixed wing.", (int) mission_waypointlist_.waypoints.size());
        exit(EXIT_FAILURE);
    }

    std::vector<geometry_msgs::PoseStamped> land_pose_vector;
    land_pose_vector.push_back(_land_pose);

    std::vector<geographic_msgs::GeoPoseStamped> usf;   // Stands for Uniformized Spatial Field
    usf = uniformizeSpatialField(land_pose_vector);

    if (usf.size() != 1) { ROS_ERROR("Error in [%d]-th waypoint set, posestamped list lenght is not 1!", (int) mission_waypointlist_.waypoints.size()); } //TODO(JoseAndres): Update errors

    mavros_msgs::Waypoint wp;
    wp = geoPoseStampedtoGlobalWaypoint(usf.back());
    wp.frame = 3;              // FRAME_GLOBAL_REL_ALT
    wp.command = 21;           // MAV_CMD_NAV_LAND
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = _abort_alt;                             // Minimum target altitude if landing is aborted (0 = undefined/use system default).
    wp.param2 = _precision_mode;                        // Precision land mode.
    wp.param4 = getYaw(usf.back().pose.orientation);    // Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).

    mission_waypointlist_.waypoints.push_back(wp);
}


void Mission::addLandWp(const geometry_msgs::PoseStamped& _loiter_to_alt_start_landing_pose, const geometry_msgs::PoseStamped& _land_pose, float _loit_radius, float _loit_heading, float _loit_forward_moving, float _abort_alt, float _precision_mode) {

    if (airframe_type_==AirframeType::MULTICOPTER || airframe_type_==AirframeType::VTOL) {
        ROS_ERROR("Error in [%d]-th waypoint, _loiter_to_alt_start_landing_pose included in Mission::addLandWp when not needed if your airframe is not a fixed wing. Please review your plan.", (int) mission_waypointlist_.waypoints.size());
        exit(EXIT_FAILURE);
    }

    std::vector<geometry_msgs::PoseStamped> land_pose_vector;
    land_pose_vector.push_back(_loiter_to_alt_start_landing_pose);
    land_pose_vector.push_back(_land_pose);

    std::vector<geographic_msgs::GeoPoseStamped> usf;   // Stands for Uniformized Spatial Field
    usf = uniformizeSpatialField(land_pose_vector);

    mavros_msgs::Waypoint wp1;
    wp1.frame = 2;              // FRAME_MISSION
    wp1.command = 189;          // MAV_CMD_DO_LAND_START
    wp1.is_current = false;
    wp1.autocontinue = true;

    mission_waypointlist_.waypoints.push_back(wp1);

    mavros_msgs::Waypoint wp2;

    if (usf.size() != 2) { ROS_ERROR("Error in [%d]-th waypoint, posestamped list length is not 2!", (int) mission_waypointlist_.waypoints.size()); }

    wp2 = geoPoseStampedtoGlobalWaypoint(usf[0]);

    wp2.frame = 3;              // FRAME_GLOBAL_REL_ALT
    wp2.command = 31;           // MAV_CMD_NAV_LOITER_TO_ALT
    wp2.is_current = false;
    wp2.autocontinue = true;
    wp2.param1 = _loit_heading;             // Heading Required. Leave loiter circle only once heading towards the next waypoint (0 = False)
    wp2.param2 = _loit_radius;              // If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
    wp2.param4 = _loit_forward_moving;      // Forward moving aircraft this sets exit xtrack location: 0 for center of loiter wp, 1 for exit location

    mission_waypointlist_.waypoints.push_back(wp2);

    mavros_msgs::Waypoint wp3;
    wp3 = geoPoseStampedtoGlobalWaypoint(usf.back());
    wp3.frame = 3;              // FRAME_GLOBAL_REL_ALT
    wp3.command = 21;           // MAV_CMD_NAV_LAND
    wp3.is_current = false;
    wp3.autocontinue = true;
    wp3.param1 = _abort_alt;                             // Minimum target altitude if landing is aborted (0 = undefined/use system default).
    wp3.param2 = _precision_mode;                        // Precision land mode.
    wp3.param4 = getYaw(usf.back().pose.orientation);    // Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).

    mission_waypointlist_.waypoints.push_back(wp3);
}


void Mission::addSpeedWp(float _speed) {

    mavros_msgs::Waypoint wp;
    wp.frame = 2;               // FRAME_MISSION
    wp.command = 178;           // MAV_CMD_DO_CHANGE_SPEED
    wp.is_current = false;
    wp.autocontinue = true;
    wp.param1 = 1.0;            // Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
    wp.param2 = _speed;         // Speed (-1 indicates no change)
    wp.param3 = -1.0;           // Throttle (-1 indicates no change)
    wp.param4 = 0.0;            // Relative (0: absolute, 1: relative)

    mission_waypointlist_.waypoints.push_back(wp);
}


void Mission::print() const {
    std::cout << std::endl;
    std::cout << "Printing the mission_waypointlist_ currently defined:" << std::endl;
    std::cout << "current_seq = " << mission_waypointlist_.current_seq << std::endl;
    std::cout << "waypoints.size() = " << mission_waypointlist_.waypoints.size() << std::endl;
    int i=0;
    for (mavros_msgs::Waypoint waypoint : mission_waypointlist_.waypoints) {
        i++;
        std::cout << std::endl;
        std::cout << "waypoints [ " << i << " ].x_lat = " << waypoint.x_lat << std::endl;
        std::cout << "waypoints [ " << i << " ].y_long = " << waypoint.y_long << std::endl;
        std::cout << "waypoints [ " << i << " ].z_alt = " << waypoint.z_alt << std::endl;
        std::cout << "waypoints [ " << i << " ].param1 = " << waypoint.param1 << std::endl;
        std::cout << "waypoints [ " << i << " ].param2 = " << waypoint.param2 << std::endl;
        std::cout << "waypoints [ " << i << " ].param3 = " << waypoint.param3 << std::endl;
        std::cout << "waypoints [ " << i << " ].param4 = " << waypoint.param4 << std::endl;
        if (waypoint.frame==0) { std::cout << "waypoints [ " << i << " ].frame = FRAME_GLOBAL" << std::endl; }
        else if (waypoint.frame==1) { std::cout << "waypoints [ " << i << " ].frame = FRAME_LOCAL_NED" << std::endl; }
        else if (waypoint.frame==2) { std::cout << "waypoints [ " << i << " ].frame = FRAME_MISSION" << std::endl; }
        else if (waypoint.frame==3) { std::cout << "waypoints [ " << i << " ].frame = FRAME_GLOBAL_REL_ALT" << std::endl; }
        else if (waypoint.frame==4) { std::cout << "waypoints [ " << i << " ].frame = FRAME_LOCAL_ENU" << std::endl; }
        std::cout << "waypoints [ " << i << " ].command = " << waypoint.command << std::endl;
        std::cout << "waypoints [ " << i << " ].is_current = " << (bool) waypoint.is_current << std::endl;
        std::cout << "waypoints [ " << i << " ].autocontinue = " << (bool) waypoint.autocontinue << std::endl;
    }
    std::cout << std::endl;
}


std::vector<geographic_msgs::GeoPoseStamped> Mission::uniformizeSpatialField( const std::vector<geometry_msgs::PoseStamped>& _posestamped_list) {

    std::vector<geographic_msgs::GeoPoseStamped> uniformized;

    for ( auto & posestamped : _posestamped_list ) {
        geographic_msgs::GeoPoseStamped homogen_world_pos;
        std::string waypoint_frame_id = tf2::getFrameId(posestamped);
        bool success = true;

        if ( waypoint_frame_id == "geo" ) {
            // No transform is needed and already in global coordinates
            homogen_world_pos.header = posestamped.header;
            homogen_world_pos.pose.orientation = posestamped.pose.orientation;
            homogen_world_pos.pose.position.latitude = posestamped.pose.position.x;
            homogen_world_pos.pose.position.longitude = posestamped.pose.position.y;
            homogen_world_pos.pose.position.altitude = posestamped.pose.position.z;
        } else if ( waypoint_frame_id == "" || waypoint_frame_id == uav_home_frame_id_ ) {
// TODO (Ángel): THIS WAS AN if, BUT SHOULD BE AN else if LIKE THIS, RIGHT? CHECK. Maybe this solves the other TODO "Check this and solve frames issue".
            // No transform is needed. Passed to global
            homogen_world_pos = poseStampedtoGeoPoseStamped(posestamped);
        } else {
            // We need to transform
            geometry_msgs::TransformStamped transformToHomeFrame;

            if ( cached_transforms_.find(waypoint_frame_id) == cached_transforms_.end() ) {
                // waypoint_frame_id not found in cached_transforms_
                try {
                    transformToHomeFrame = tf_buffer_.lookupTransform(uav_home_frame_id_, waypoint_frame_id, ros::Time(0), ros::Duration(1.0));
                    cached_transforms_[waypoint_frame_id] = transformToHomeFrame; // Save transform in cache
                } catch (tf2::TransformException &ex) {
                    ROS_ERROR("Transformation not found");
                    success = false;
                }
            } else {
                // found in cache
                transformToHomeFrame = cached_transforms_[waypoint_frame_id];
            }
            geometry_msgs::PoseStamped aux = posestamped;
            tf2::doTransform(posestamped, aux, transformToHomeFrame);
            homogen_world_pos = poseStampedtoGeoPoseStamped(aux);
        }

        uniformized.push_back( homogen_world_pos);

    }

    return uniformized;
}


geographic_msgs::GeoPoseStamped Mission::poseStampedtoGeoPoseStamped(const geometry_msgs::PoseStamped& _posestamped ) {

    geometry_msgs::Point32 geo_point;
    geo_point.x = _posestamped.pose.position.x;
    geo_point.y = _posestamped.pose.position.y;
    geo_point.z = _posestamped.pose.position.z;

    geographic_msgs::GeoPoint actual_geo = cartesian_to_geographic(geo_point, origin_geo_);

    geographic_msgs::GeoPoseStamped geopose;
    geopose.pose.position.latitude = actual_geo.latitude;
    geopose.pose.position.longitude = actual_geo.longitude;
    geopose.pose.position.altitude = actual_geo.altitude;
    geopose.header = _posestamped.header;
    geopose.pose.orientation = _posestamped.pose.orientation;

    return geopose;
}


geometry_msgs::PoseStamped Mission::geoPoseStampedtoPoseStamped(const geographic_msgs::GeoPoseStamped _geoposestamped ) {

    geographic_msgs::GeoPoint aux;
    aux.latitude = _geoposestamped.pose.position.latitude;
    aux.longitude = _geoposestamped.pose.position.longitude;
    aux.altitude = _geoposestamped.pose.position.altitude;

    geometry_msgs::Point32 actual_geo = geographic_to_cartesian(aux, origin_geo_);

    geometry_msgs::PoseStamped posestamped;
    posestamped.pose.position.x = actual_geo.x;
    posestamped.pose.position.y = actual_geo.y;
    posestamped.pose.position.z = actual_geo.z;
    posestamped.header = _geoposestamped.header;
    posestamped.pose.orientation = _geoposestamped.pose.orientation;

    return posestamped;
}


mavros_msgs::Waypoint Mission::geoPoseStampedtoGlobalWaypoint(const geographic_msgs::GeoPoseStamped& _geoposestamped ) {

    mavros_msgs::Waypoint waypoint;
    waypoint.x_lat = _geoposestamped.pose.position.latitude;
    waypoint.y_long = _geoposestamped.pose.position.longitude;
    waypoint.z_alt = _geoposestamped.pose.position.altitude;

    return waypoint;
}


float Mission::getYaw(const geometry_msgs::Quaternion& _quat) {

    float yaw;

    double norm = _quat.x*_quat.x  + _quat.y*_quat.y  + _quat.z*_quat.z  + _quat.w*_quat.w;

    if ( norm == 0) {
        yaw = std::nanf("0.0");
    } else {
        yaw = tf::getYaw(_quat);
    }

    return yaw;
}

}}	// namespace grvc::mission_ns