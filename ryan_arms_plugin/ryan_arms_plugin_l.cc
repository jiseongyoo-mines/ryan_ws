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
    // Ryam left arms
    private: physics::JointPtr joint1, joint3, joint5, joint7;
    private: common::PID pid1, pid3, pid5, pid7;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    public: RyanArmsPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      std::cerr << "\nThe RyanArms plugin for left arms is attach to model[" <<
        _model->GetName() <<"]\n";

      // Safety check
      std::cerr << "The number of joints: " << _model->GetJointCount() << "\n";
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, RyanArms plugin not loaded\n";
        return;
      }

      this->model = _model;

      this->joint1 = _model->GetJoints()[1];   // left shoulder joint x axis
      this->joint3 = _model->GetJoints()[3];   // left shoulder joint y axis
      this->joint5 = _model->GetJoints()[5];   // left elbow joint x axis
      this->joint7 = _model->GetJoints()[7];   // left elbow joint z axis *
      this->pid1 = common::PID(50, 0, 3);      // Change Kp, Ki, Kd
      this->pid3 = common::PID(20, 10, 3);     // Change Kp, Ki, Kd
      this->pid5 = common::PID(280, 100, 3);   // Change Kp, Ki, Kd
      this->pid7 = common::PID(8, 0, 3);       // Change Kp, Ki, Kd

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetPositionPID(
          this->joint1->GetScopedName(), this->pid1);
      this->model->GetJointController()->SetPositionPID(
          this->joint3->GetScopedName(), this->pid3);
      this->model->GetJointController()->SetPositionPID(
          this->joint5->GetScopedName(), this->pid5);
      this->model->GetJointController()->SetPositionPID(
          this->joint7->GetScopedName(), this->pid7);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->model->GetWorld()->GetName());
      #else
        this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/left_arms_pos_cmd";

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
            "/" + this->model->GetName() + "/left_pos_cmd", 1,
           boost::bind(&RyanArmsPlugin::OnRosMsgLeft, this, _1),
           ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
      std::thread(std::bind(&RyanArmsPlugin::QueueThread, this));
    }

    // Set the position of ryan arm
    public: void SetPosition(physics::JointPtr &_joint, const double _pos)
    {
      // Set the joint's target position.
      this->model->GetJointController()->SetPositionTarget(
          _joint->GetScopedName(), _pos);
    }

    // Set the velocity of ryan arm
    public: void SetVelocity(physics::JointPtr &_joint, const double &_vel)
    {
      // Set the joint's target velocity.
      this->model->GetJointController()->SetVelocityTarget(
          _joint->GetScopedName(), _vel);
    }

    public: void OnRosMsgLeft(const std_msgs::Float32ConstPtr &_msg)
    {
      std::cout << "Left arm commands\n";
      // this->SetPosition(this->joint1, _msg->data);
      // this->SetPosition(this->joint3, _msg->data);
      // this->SetPosition(this->joint5, _msg->data);
      // this->SetPosition(this->joint7, _msg->data);
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
