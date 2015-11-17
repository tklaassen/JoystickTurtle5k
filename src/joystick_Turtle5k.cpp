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


