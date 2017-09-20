//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
// Based on John Hsu's GazeboRosJointTrajectory plugin
//----------------------------------------------------------------------------------------------------------------------
#ifndef GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
 #define GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
 
 #include <ros/ros.h>
 #include <ros/callback_queue.h>
 #include <ros/advertise_options.h>
 #include <ros/subscribe_options.h>
 #include <sensor_msgs/JointState.h>
 #include <geometry_msgs/Pose.h>
 #include <gazebo/physics/physics.hh>
 #include <gazebo/transport/TransportTypes.hh>
 #include <gazebo/common/Time.hh>
 #include <gazebo/common/Plugin.hh>
 #include <gazebo/common/Events.hh>
 
 #include <boost/thread.hpp>
 #include <boost/thread/mutex.hpp>
 
 namespace gazebo {
 
class JointAnimatorGazeboPlugin : public ModelPlugin {
    /// \brief Constructor
    public: JointAnimatorGazeboPlugin();

    /// \brief Destructor
    public: virtual ~JointAnimatorGazeboPlugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    private: void SetJointState(const sensor_msgs::JointState::ConstPtr& _joint_state);

    private: void UpdateStates();

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// \brief pose should be set relative to this link (default to "world")
    private: physics::LinkPtr reference_link_;
    private: std::string reference_link_name_;
    /// \brief frame transform name, should match link name
    //private: std::string tf_frame_name_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Subscriber sub_;
//    private: ros::ServiceServer srv_;
//    private: bool has_trajectory_;

    /// \brief ros message
//    private: trajectory_msgs::JointTrajectory trajectory_msg_;
//    private: bool set_model_pose_;
//    private: geometry_msgs::Pose model_pose_;

    /// \brief topic name
    private: std::string topic_name_;
//    private: std::string service_name_;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex update_mutex;

    /// \brief save last_time
    private: common::Time last_time_;

    // trajectory time control
//    private: common::Time trajectory_start;
//    private: unsigned int trajectory_index;

    // rate control
    private: double update_rate_;
    private: bool disable_physics_updates_;
    private: bool physics_engine_enabled_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    private: std::vector<gazebo::physics::JointPtr> joints_;
    private: std::vector<double> positions_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;
}; 

}  // namespace gazebo
#endif  // GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
