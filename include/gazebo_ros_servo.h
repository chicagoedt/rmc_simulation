#ifndef GAZEBO_ROS_SERVO_HH
#define GAZEBO_ROS_SERVO_HH

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <rmc_simulation/PanServoAction.h>
#include <actionlib/server/simple_action_server.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

   class GazeboRosServo : public ModelPlugin
   {
      /// \brief Constructor
      public: GazeboRosServo();

      /// \brief Destructor
      public: virtual ~GazeboRosServo();

      /// \brief Load the controller
      public:  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      /// \brief Update the controller
      //protected: virtual void UpdateChild();
      public:  void OnUpdate(const common::UpdateInfo & _info);

   private:
      physics::ModelPtr _model;
      event::ConnectionPtr updateConnection;

      ros::NodeHandle _nh;
      ros::Publisher servoRotationPub_;
      ros::Subscriber sub_;
      tf::TransformBroadcaster *transform_broadcaster_;
      tf::TransformListener* transform_listener_;

      physics::JointPtr _servoJoint;

      common::Time pastTime;
      double timeInc;

      double _frequencyUpdate;

      std::string _cameraFrameId;
      std::string _robotFrameId;

      std::string _cameraJointName;

      double _rotatingVelocity;

      double _maxForce;

      gazebo::math::Angle _jointUpperLimit;
      gazebo::math::Angle _jointLowerLimit;
      gazebo::math::Angle _currentAngle;

      bool  _leftLimitReached;
      bool  _rightLimitReached;
      bool  _donePanning;

      actionlib::SimpleActionServer<rmc_simulation::PanServoAction>* _server;

   };

}

#endif
