//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
// Based on John Hsu's GazeboRosJointTrajectory plugin
//----------------------------------------------------------------------------------------------------------------------
#ifndef GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
#define GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H

 #include <ros/ros.h>
 #include <ros/callback_queue.h>
 #include <ros/subscribe_options.h>
 #include <sensor_msgs/JointState.h>
 #include <gazebo/physics/physics.hh>
 #include <gazebo/common/Time.hh>
 #include <gazebo/common/Plugin.hh>
 #include <gazebo/common/Events.hh>
 #include <boost/thread.hpp>
 #include <boost/thread/mutex.hpp>

namespace gazebo {

class JointAnimatorGazeboPlugin : public ModelPlugin {
public:

    JointAnimatorGazeboPlugin();

    virtual ~JointAnimatorGazeboPlugin();

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:

    void SetJointState(const sensor_msgs::JointState::ConstPtr& _joint_state);

    void UpdateStates();

    physics::WorldPtr world_;
    physics::ModelPtr model_;

    /// \brief pose should be set relative to this link (default to "world")
    physics::LinkPtr reference_link_;
    std::string reference_link_name_;

    /// \brief pointer to ros node
    ros::NodeHandle* rosnode_;
    ros::Subscriber sub_;

    /// \brief topic name
    std::string topic_name_;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    boost::mutex update_mutex;

    /// \brief save last_time
    common::Time last_time_;

    // rate control
    double update_rate_;
    bool disable_physics_updates_;
    bool physics_engine_enabled_;

    /// \brief for setting ROS name space
    std::string robot_namespace_;

    ros::CallbackQueue queue_;
    void QueueThread();
    boost::thread callback_queue_thread_;

    std::vector<gazebo::physics::JointPtr> joints_;
    std::vector<double> positions_;

    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;
}; 

}  // namespace gazebo
#endif  // GAZEBO_ANIMATOR_GAZEBO_PLUGIN_H
