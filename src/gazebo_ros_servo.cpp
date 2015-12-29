#include <gazebo_ros_servo.h>
#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosServo);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosServo::GazeboRosServo(): _leftLimitReached(false), _rightLimitReached(false)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosServo::~GazeboRosServo()
{
	if(rosnode_)
		delete rosnode_;

	if(transform_listener_)
		delete transform_listener_;

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

    this->_frequencyUpdate = 10; 
    if (!_sdf->HasElement("updateRate")) 
    {
		ROS_WARN("GazeboServoPlugin is missing <updateRate>, defaults to 10");
    } 
    else 
    {
		this->_frequencyUpdate = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->_cameraFrameId = "blackfly_mount_link"; 
    if (!_sdf->HasElement("cameraFrameName")) 
    {
		ROS_WARN("GazeboServoPlugin is missing <cameraFrameName>, defaults to \"%s\"", this->_cameraFrameId.c_str());
    } 
    else 
    {
		this->_cameraFrameId = _sdf->GetElement("cameraFrameName")->Get<std::string>();
    }

    this->_robotFrameId = "base_link"; 
    if (!_sdf->HasElement("robotBaseFrameName")) 
    {
		ROS_WARN("GazeboServoPlugin is missing <robotBaseFrameName>, defaults to \"%s\"", this->_robotFrameId.c_str());
    } 
    else 
    {
		this->_robotFrameId = _sdf->GetElement("robotBaseFrameName")->Get<std::string>();
    }

    this->_cameraJointName = "blackfly_mount_joint"; 
    if (!_sdf->HasElement("cameraJointName")) 
    {
		ROS_WARN("GazeboServoPlugin is missing <cameraJointName>, defaults to \"%s\"", this->_cameraJointName.c_str());
    } 
    else 
    {
		this->_cameraJointName = _sdf->GetElement("cameraJointName")->Get<std::string>();
    }

    this->_rotatingVelocity = 0.9; 
    if (!_sdf->HasElement("rotatingVelocity")) 
    {
		ROS_WARN("GazeboServoPlugin is missing <rotatingVelocity>, defaults to 0.9");
    } 
    else 
    {
		this->_rotatingVelocity = _sdf->GetElement("rotatingVelocity")->Get<double>();
    }

    this->_maxForce = 20; 
    if (!_sdf->HasElement("maxForceNewtonMeters")) 
    {
		ROS_WARN("GazeboServoPlugin is missing <maxForceNewtonMeters>, defaults to 20");
    } 
    else 
    {
		this->_maxForce = _sdf->GetElement("maxForceNewtonMeters")->Get<double>();
    }


  	this->rosnode_ = new ros::NodeHandle();

  	transform_listener_ = new tf::TransformListener();
  	transform_listener_->setExtrapolationLimit(ros::Duration(1.0));

  	this->timeInc = 0;
    this->pastTime = 0;

  	this->_model = _model;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
								boost::bind(&GazeboRosServo::OnUpdate, this, _1));

  	this->_servoJoint = this->_model->GetJoint(this->_cameraJointName);
  	this->_servoJoint->SetMaxForce(0, this->_maxForce);

  	_jointUpperLimit = this->_servoJoint->GetUpperLimit(0);
  	_jointLowerLimit = this->_servoJoint->GetLowerLimit(0);

  	ROS_INFO_STREAM("Joint Angle: " << this->_servoJoint->GetAngle(0));

}

void GazeboRosServo::OnUpdate(const common::UpdateInfo & _info)
{
    common::Time currentTime = _info.simTime;
	double timeDifference = currentTime.Double()-this->pastTime.Double();

	if (timeDifference < (1/_frequencyUpdate)) 
		return;
/*
	tf::StampedTransform transform;

	try
	{
		transform_listener_->lookupTransform(this->_cameraFrameId, this->_robotFrameId, ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_WARN_NAMED("servo_plugin","%s",ex.what());
		return;
	}
*/

	// TODO: dont compare double directly
	if(!this->_leftLimitReached)
	{
		ROS_INFO("LEFT");
		this->_servoJoint->SetVelocity(0, this->_rotatingVelocity);
		if(this->_servoJoint->GetAngle(0) == _jointUpperLimit)
		{
			ROS_INFO("Left Panning Limit Reached! Rotating otherway...");
			_leftLimitReached = true;
			this->_servoJoint->SetVelocity(0, 0); // Temporarily stop applying velocity
			// Need to set angle to current angle when we reach upper limit, or else when we switch
			// rotational directions, the joint will jump to it starting angle and then start rotating.
			this->_servoJoint->SetAngle(0, _jointUpperLimit); 
		}
		else
		{
			this->_servoJoint->SetAngle(0, this->_servoJoint->GetAngle(0));
		}
		ROS_INFO_STREAM("Joint Angle1: " << this->_servoJoint->GetAngle(0));
		
	}
	else if(!this->_rightLimitReached)
	{
		//_servoJoint->SetMaxForce(0, -this->_maxForce);
		ROS_INFO("RIGHT");

		this->_servoJoint->SetVelocity(0, -this->_rotatingVelocity);
		if(this->_servoJoint->GetAngle(0) == _jointLowerLimit)
		{
			ROS_INFO("Right Panning Limit Reached! Did not find marker!");
			_rightLimitReached = true;	
			
			this->_servoJoint->SetVelocity(0, 0);

			ROS_INFO_STREAM("Before Angle: " << this->_servoJoint->GetAngle(0));
			this->_servoJoint->SetAngle(0, _jointLowerLimit);
			ROS_INFO_STREAM("After Angle: " << this->_servoJoint->GetAngle(0));
			//this->_servoJoint->Update();

		}
		else
		{
			this->_servoJoint->SetAngle(0, this->_servoJoint->GetAngle(0));
		}
		ROS_INFO_STREAM("Joint Angle2: " << this->_servoJoint->GetAngle(0));
	}
	else
	{
		this->_servoJoint->SetVelocity(0, this->_rotatingVelocity);
		ROS_INFO("Vel = +");
		if(this->_servoJoint->GetAngle(0) > 0)
		{
			ROS_INFO("Vel >= 0");
			ROS_INFO_STREAM("Angle "<< _servoJoint->GetAngle(0));
			//ROS_INFO("Servo back at 0, waiting for request to pan again...");
			// TODO: set limitReached bool's to false once state machine performs robot rotation
			this->_servoJoint->SetVelocity(0, 0.0);	
			_servoJoint->SetAngle(0, 0.0);
		}
		else
		{
			//this->_servoJoint->SetVelocity(0, this->_rotatingVelocity);
			this->_servoJoint->SetAngle(0, this->_servoJoint->GetAngle(0)); // negative velocity
		}
	}

	//ROS_INFO_STREAM("After3 Angle: " << this->_servoJoint->GetAngle(0));

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
