//----------------------------------------------------------------------------------------------------------------------
// GRVC Utils
// Based on John Hsu's GazeboRosJointTrajectory plugin
//----------------------------------------------------------------------------------------------------------------------
#include "gazebo_animator/joint_animator_gazebo_plugin.h"
// #include "tf/tf.h"
 
namespace gazebo {

JointAnimatorGazeboPlugin::JointAnimatorGazeboPlugin() {
    // this->has_trajectory_ = false;
    // this->trajectory_index = 0;
    // this->joint_trajectory_.points.clear();
    this->physics_engine_enabled_ = true;
    this->disable_physics_updates_ = true;
}

JointAnimatorGazeboPlugin::~JointAnimatorGazeboPlugin() {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
    // Finalize the controller
    this->rosnode_->shutdown();
    this->queue_.clear();
    this->queue_.disable();
    this->callback_queue_thread_.join();
    delete this->rosnode_;
}

void JointAnimatorGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model_ = _model;

    // Get the world
    this->world_ = _model->GetWorld();

    //this->world_->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));

    // load parameters
    this->robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace")) {
        this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString() + "/";
    }

    // if (!_sdf->HasElement("serviceName")) {
    //     this->service_name_ = "set_joint_trajectory";
    // } else {
    //     this->service_name_ = _sdf->GetElement("serviceName")->GetValueString();
    // }
        
    if (!_sdf->HasElement("topicName")) {
        this->topic_name_ = "set_joint_state";
    } else {
        this->topic_name_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
    }

    if (!_sdf->HasElement("updateRate")) {
        ROS_INFO("joint animator plugin missing <updateRate>, defaults to 0.0 (as fast as possible)");
        this->update_rate_ = 0;
    } else {
        _sdf->GetElement("updateRate")->GetValue()->Get(this->update_rate_);
    }

    // Wait for ROS
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = NULL;
        ros::init( argc, argv, "gazebo", ros::init_options::NoSigintHandler);
        gzwarn << "should start ros::init in simulation by using the system plugin\n";
    }

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    // resolve tf prefix
    std::string prefix;
    this->rosnode_->getParam(std::string("tf_prefix"), prefix);

    if (this->topic_name_ != "") {
        ros::SubscribeOptions joint_state_so = ros::SubscribeOptions::create<sensor_msgs::JointState>(
            this->topic_name_, 10, boost::bind(&JointAnimatorGazeboPlugin::SetJointState, this, _1),
            ros::VoidPtr(), &this->queue_);
        this->sub_ = this->rosnode_->subscribe(joint_state_so);
    }

    this->last_time_ = this->world_->GetSimTime();

    // start custom queue for joint state plugin ros topics
    this->callback_queue_thread_ = boost::thread(boost::bind(&JointAnimatorGazeboPlugin::QueueThread, this));

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointAnimatorGazeboPlugin::UpdateStates, this));
}

void JointAnimatorGazeboPlugin::SetJointState(const sensor_msgs::JointState::ConstPtr& _joint_state) {
    boost::mutex::scoped_lock lock(this->update_mutex);

    // resume physics update
    this->world_->EnablePhysicsEngine(this->physics_engine_enabled_);

    // this->reference_link_name_ = trajectory->header.frame_id;
    // // do this every time a new joint trajectory is supplied, use header.frame_id as the reference_link_name_
    // if (this->reference_link_name_ != "world" && this->reference_link_name_ != "/map" && this->reference_link_name_ != "map") {
    //     physics::EntityPtr ent = this->world_->GetEntity(this->reference_link_name_);
    //     if (ent) {
    //         this->reference_link_ = boost::shared_dynamic_cast<physics::Link>(ent);    
    //     }
    //     if (!this->reference_link_) {
    //         ROS_ERROR("ros_joint_trajectory plugin needs a reference link [%s] as frame_id, aborting.\n",
    //         this->reference_link_name_.c_str());
    //         return;
    //     } else {
    //         this->model_ = this->reference_link_->GetParentModel();
    //         ROS_DEBUG("test: update model pose by keeping link [%s] stationary inertially",this->reference_link_->GetName().c_str());
    //     }
    // }

    // copy joint configuration into a map
    unsigned int chain_size = _joint_state->name.size();
    this->joints_.resize(chain_size);
    this->positions_.resize(chain_size);
    for (unsigned int i = 0; i < chain_size; ++i) {
        this->joints_[i] = this->model_->GetJoint(_joint_state->name[i]);
        this->positions_[i] = _joint_state->position[i];
    }

    // unsigned int points_size = trajectory->points.size();
    // this->points_.resize(points_size);
    // for (unsigned int i = 0; i < points_size; ++i)
    // {
    //     this->points_[i].positions.resize(chain_size);
    //     for (unsigned int j = 0; j < chain_size; ++j)
    //     {
    //     this->points_[i].positions[j] = trajectory->points[i].positions[j];
    //     }
    // }

    // // trajectory start time
    // this->trajectory_start = gazebo::common::Time(trajectory->header.stamp.sec,
    //                                                 trajectory->header.stamp.nsec);
    // common::Time cur_time = this->world_->GetSimTime();
    // if (this->trajectory_start < cur_time) {
    //     this->trajectory_start = cur_time;
    // }
        
    // update the joint trajectory to play
    // this->has_trajectory_ = true;
    // reset trajectory_index to beginning of new trajectory
    // this->trajectory_index = 0;

    if (this->disable_physics_updates_) {
        this->physics_engine_enabled_ = this->world_->GetEnablePhysicsEngine();
        this->world_->EnablePhysicsEngine(false);
    }
}


void JointAnimatorGazeboPlugin::UpdateStates() {

    boost::mutex::scoped_lock lock(this->update_mutex);
    common::Time cur_time = this->world_->GetSimTime();

    // get reference link pose before updates
    math::Pose reference_pose = this->model_->GetWorldPose();
    if (this->reference_link_) {
        reference_pose = this->reference_link_->GetWorldPose();
    }

    // trajectory roll-out based on time:  set model configuration from trajectory message
    for (unsigned int i = 0; i < this->joints_.size(); ++i) {
        // this is not the most efficient way to set things
        if (this->joints_[i]) {
            this->joints_[i]->SetPosition(0, this->positions_[i]);
        }
    }

    // set model pose
    if (this->reference_link_) {
        this->model_->SetLinkWorldPose(reference_pose, this->reference_link_);
    } else {
        this->model_->SetWorldPose(reference_pose);
    }
    //this->world_->SetPaused(is_paused); // resume original pause-state
    // gazebo::common::Time duration(this->points_[this->trajectory_index].time_from_start.sec,
    //                             this->points_[this->trajectory_index].time_from_start.nsec);
    // this->trajectory_start += duration; // reset start time for next trajectory point
    // this->trajectory_index++; // increment to next trajectory point

    // save last update time stamp
    this->last_time_ = cur_time;


    // no more trajectory points
// {
//     // trajectory finished
    this->reference_link_.reset();
    // this->has_trajectory_ = false;
    if (this->disable_physics_updates_)
    this->world_->EnablePhysicsEngine(this->physics_engine_enabled_);
// }

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void JointAnimatorGazeboPlugin::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosnode_->ok()) {
        this->queue_.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_MODEL_PLUGIN(JointAnimatorGazeboPlugin);

}
