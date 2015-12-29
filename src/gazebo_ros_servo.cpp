#include <gazebo_ros_servo.h>
#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosServo);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosServo::GazeboRosServo()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosServo::~GazeboRosServo()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosServo::Load( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
  	// Make sure the ROS node for Gazebo has already been initalized
  	if (!ros::isInitialized())
  	{
    	ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      	<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    	return;
  	}
  	this->rosnode_ = new ros::NodeHandle();

  	transform_listener_ = new tf::TransformListener();
  	transform_listener_->setExtrapolationLimit(ros::Duration(1.0));

  	this->timeInc = 0;
    this->pastTime = 0;

  	this->_model = _model;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
								boost::bind(&GazeboRosServo::OnUpdate, this, _1));

  	_servoJoint = this->_model->GetJoint("blackfly_mount_joint");
  	_servoJoint->SetMaxForce(0, 20);

  	ROS_INFO_STREAM("Joint Angle: " << _servoJoint->GetAngle(0));

  	ROS_INFO("HELLO");



}

void GazeboRosServo::OnUpdate(const common::UpdateInfo & _info)
{
    common::Time currentTime = _info.simTime;
	double timeDifference = currentTime.Double()-pastTime.Double();
	if (timeDifference < (1/frequencyUpdate)) return;

	tf::StampedTransform transform;

	ROS_INFO("HELLO");

	try
	{
		transform_listener_->lookupTransform("blackfly_mount_link", "base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN_NAMED("servo_plugin","%s",ex.what());
		return;
	}

	ROS_INFO_STREAM("Joint Angle: " << _servoJoint->GetAngle(0));
	_servoJoint->SetVelocity(0, 1.0);

	this->pastTime = currentTime;
	
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
/*
void GazeboRosServo::UpdateChild()
{

}
*/
}
