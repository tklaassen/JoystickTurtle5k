// includes
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>

//create class motordrive, functions and variables
class MotorDriver123
{
public:
  MotorDriver123();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int iMotor1, iMotor2, iMotor3;
  double dScaleMotor1, dScaleMotor2, dScaleMotor3;

  ros::Publisher pub_VelPub;
  ros::Subscriber sub_VelSub;
  
};

// receive output Joystick from parameter file and set parameters
MotorDriver123::MotorDriver123():
  iMotor1(1),
  iMotor2(2),
  iMotor3(3)
{

	nh_.param("AxisMotor1", iMotor1, iMotor1);
	nh_.param("AxisMotor2", iMotor2, iMotor2);
	nh_.param("AxisMotor3", iMotor3, iMotor3);

	nh_.param("dScaleMotor1", dScaleMotor1, dScaleMotor1);
	nh_.param("dScaleMotor2", dScaleMotor2, dScaleMotor2);
	nh_.param("dScaleMotor3", dScaleMotor3, dScaleMotor3);


// Publisch mutliarray with motorspeed
  pub_VelPub = nh_.advertise<std_msgs::Float32MultiArray>("mcWheelVelocityMps", 1000);

// Read joy toppic
  sub_VelSub = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &MotorDriver123::joyCallback, this);

}
// set speed values and put them into a multiarray
void MotorDriver123::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//create some variables
	double dVelMotor1, dVelMotor2, dVelMotor3;
	ros::Rate loop_rate(100);

while(ros::ok())
	{
	//Check params Motor 1, 2, 3
		if (nh_.hasParam("AxisMotor1"),nh_.hasParam("AxisMotor2"), nh_.hasParam("AxisMotor3"))
		{
			ROS_INFO_ONCE("The motorparameters are initialized with values: Motor1: %i, Motor2: %i, Motor3: %i.", iMotor1, iMotor2, iMotor3);
		}
		else
		{
			ROS_ERROR_ONCE("The motorparameter(s) cannot be initialized");
			iMotor1=0; iMotor2=0, iMotor3=0;
		}

	//Check params scaler 1, 2, 3
		if (nh_.hasParam("dScaleMotor1"),nh_.hasParam("dScaleMotor2"),nh_.hasParam("dScaleMotor3"))
		{
			ROS_INFO_ONCE("The scalerparameters are initialized with values: Scaler1: %f, Scaler2: %f, Scaler3: %f.", dScaleMotor1, dScaleMotor2, dScaleMotor3);
		}
		else
		{
			ROS_ERROR_ONCE("The scalerparameter(s) cannot be initialized");
			dScaleMotor1=0; dScaleMotor2=0, dScaleMotor3=0;
		}

	// calculate speedvalues
		dVelMotor1 = dScaleMotor1*joy->axes[iMotor1];
		dVelMotor2 = dScaleMotor2*joy->axes[iMotor2];
		dVelMotor3 = dScaleMotor3*joy->axes[iMotor3];

	//create multiarray
		std_msgs::Float32MultiArray msg;

	//put speedvalues into array
		msg.data.clear();
		msg.data.push_back(0);
		msg.data.push_back(0);
		msg.data.push_back(0);
		msg.data.push_back(0);
		msg.data.push_back(dVelMotor1);
		msg.data.push_back(0);
		msg.data.push_back(dVelMotor2);
		msg.data.push_back(dVelMotor3);
		msg.data.push_back(0);
		msg.data.push_back(0);

		pub_VelPub.publish(msg);

			ROS_DEBUG("Actual motor set state: Motor1: %f, Motor2: %f, Motor3: %f.", dVelMotor1, dVelMotor2, dVelMotor3);
			ROS_DEBUG("Actual scaler values: Scaler1: %f, Scaler2: %f, Scaler3: %f.", dScaleMotor1, dScaleMotor2, dScaleMotor3);

		ros::spinOnce();
		loop_rate.sleep();

	}

}

// Main
int main(int argc, char** argv)
	{

		
  		ros::init(argc, argv, "MotorDrivers");

  		MotorDriver123 MotorDrivers;

  		ros::spin();
	}


