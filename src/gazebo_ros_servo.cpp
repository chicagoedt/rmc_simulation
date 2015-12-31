#include <gazebo_ros_servo.h>
#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosServo);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosServo::GazeboRosServo(): _leftLimitReached(false), _rightLimitReached(false), _donePanning(true)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosServo::~GazeboRosServo()
{
	if(transform_listener_)
		delete transform_listener_;

	this->_server->shutdown();
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


  	//this->_nh = new ros::NodeHandle();

  	transform_listener_ = new tf::TransformListener();
  	transform_listener_->setExtrapolationLimit(ros::Duration(1.0));

  	transform_broadcaster_ = new tf::TransformBroadcaster();

  	this->_server = new actionlib::SimpleActionServer<rmc_simulation::PanServoAction>(_nh, "pan_servo", false);
  	this->_server->start();
  	

  	this->timeInc = 0;
    this->pastTime = 0;

  	this->_model = _model;

	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
								boost::bind(&GazeboRosServo::OnUpdate, this, _1));

  	this->_servoJoint = this->_model->GetJoint(this->_cameraJointName);
  	this->_servoJoint->SetMaxForce(0, this->_maxForce);

  	this->_cameraJoint = this->_model->GetJoint("blackfly_optical_joint");

  	_jointUpperLimit = this->_servoJoint->GetUpperLimit(0);
  	_jointLowerLimit = this->_servoJoint->GetLowerLimit(0);
  	_currentAngle = gazebo::math::Angle(0);

  	//ROS_INFO_STREAM("Joint Angle: " << this->_servoJoint->GetAngle(0));

}



void GazeboRosServo::OnUpdate(const common::UpdateInfo & _info)
{
	if(this->_server->isNewGoalAvailable())
	{
		boost::shared_ptr<const rmc_simulation::PanServoGoal> goal = this->_server->acceptNewGoal();
		ROS_INFO("New Goal, not active...");
		_donePanning = false;
	}
	
	gazebo::math::Pose servoPose = this->_servoJoint->GetChild()->GetRelativePose();

	tf::Quaternion servoYaw(servoPose.rot.x, servoPose.rot.y, servoPose.rot.z, servoPose.rot.w);
	tf::Vector3 servoXYZ (servoPose.pos.x, servoPose.pos.y, servoPose.pos.z);

	//tf::Quaternion cameraOpticalRotation;
	//cameraOpticalRotation.setRPY(0, 1.57, 1.57);

	//servoYaw += cameraOpticalRotation;

	tf::Transform tfWheel(servoYaw, servoXYZ);

	transform_broadcaster_->sendTransform(
        tf::StampedTransform(tfWheel, ros::Time::now(), "base_link", _cameraFrameId));
	
	if(!_donePanning)
	{

	    common::Time currentTime = _info.simTime;
		double timeDifference = currentTime.Double()-this->pastTime.Double();



	/*
		if (timeDifference < (1/_frequencyUpdate)) 
			return;

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
		if(!this->_leftLimitReached)
		{
			//ROS_INFO("LEFT");
			this->_servoJoint->SetAngle(0, (this->_servoJoint->GetAngle(0) + (this->_rotatingVelocity * timeDifference)));
			if(this->_servoJoint->GetAngle(0) == _jointUpperLimit)
			{
				//ROS_INFO("Left Limit Reached...");
				this->_leftLimitReached = true;
			}
		}
		else if(!this->_rightLimitReached)
		{
			//ROS_INFO("RIGHT");
			this->_servoJoint->SetAngle(0, (this->_servoJoint->GetAngle(0) - (this->_rotatingVelocity * timeDifference)));
			if(this->_servoJoint->GetAngle(0) == _jointLowerLimit)
			{
				//ROS_INFO("Right Limit Reached...");
				this->_rightLimitReached = true;	
			}
		}
		else
		{
			//ROS_INFO("CENTER");
			this->_servoJoint->SetAngle(0, (this->_servoJoint->GetAngle(0) + (this->_rotatingVelocity * timeDifference)));
			if(this->_servoJoint->GetAngle(0) > 0)
			{
				this->_servoJoint->SetAngle(0, 0);
				//ROS_INFO("Center Reached...");
				_donePanning = true;
				//this->_server->acceptNewGoal();
			}
		}

		if (_server->isPreemptRequested())
		{
			ROS_INFO("Preemptive Goal requested... Stopping servo rotation.");
			_server->setPreempted();
			this->_rightLimitReached = false;
			this->_leftLimitReached = false;
		}
		else if(_donePanning)
		{
			ROS_INFO("Sending succeeded message...");
			rmc_simulation::PanServoResult result;
			result.completed_panning = true;

			this->_server->setSucceeded(result);
			this->_rightLimitReached = false;
			this->_leftLimitReached = false;
		}

		//ROS_INFO_STREAM("After3 Angle: " << this->_servoJoint->GetAngle(0));

		this->pastTime = currentTime;
		ros::spinOnce();
	}
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
/*
void GazeboRosServo::UpdateChild()
{

}
*/
}
