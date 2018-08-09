#ifndef _RYAN_ARMS_PLUGIN_HH_
#define _RYAN_ARMS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class RyanArmsPlugin : public ModelPlugin
  {
    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: common::PID pid;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    public: RyanArmsPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      std::cerr << "\nThe RyanArms plugin is attach to model[" <<
        _model->GetName() <<"]\n";

      // Safety check
      std::cerr << "The number of joints: " << _model->GetJointCount() << "\n";
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      this->model = _model;
      this->joint = _model->GetJoints()[1];   // left shoulder joint x axis
      this->pid = common::PID(8, 0, 3);   // Change Kp, Ki, Kd

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetPositionPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero position
      double position = 0;

      // Check that the position element exists, then read the value
      if (_sdf->HasElement("position"))
        position = _sdf->Get<double>("position");

      // Set the joint's target position.
      this->model->GetJointController()->SetPositionTarget(
          this->joint->GetScopedName(), position);


      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
      #else
        this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/pos_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &RyanArmsPlugin::OnMsg, this);

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/pos_cmd", 1,
           boost::bind(&RyanArmsPlugin::OnRosMsg, this, _1),
           ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      // Spin up the queue helper thread.
      this->rosQueueThread =
      std::thread(std::bind(&RyanArmsPlugin::QueueThread, this));
    }

    // Set the position of ryan arm
    public: void SetPosition(const double &_pos)
    {
      // Set the joint's target position.
      this->model->GetJointController()->SetPositionTarget(
          this->joint->GetScopedName(), _pos);
    }

    // Set the velocity of ryan arm
    public: void SetVelocity(const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          this->joint->GetScopedName(), _vel);
    }

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
      this->SetPosition(_msg->x());
    }

    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetPosition(_msg->data);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(RyanArmsPlugin)
}

#endif
