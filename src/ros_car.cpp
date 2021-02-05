#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>

std_msgs::Int32 servo_msg;
std_msgs::Int32 motor_msg;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  servo_msg.data = -joy_msg.axes[2] * 25 + 90;

  if (joy_msg.axes[1] > 0.5)
    motor_msg.data = 1;
  else if (joy_msg.axes[1] < -0.5)
    motor_msg.data = -1;
  else
    motor_msg.data = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_car_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);
  ros::Publisher pub_servo = nh.advertise<std_msgs::Int32>("servo", 10);
  ros::Publisher pub_motor = nh.advertise<std_msgs::Int32>("motor", 10);
  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    pub_servo.publish(servo_msg);
    pub_motor.publish(motor_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
