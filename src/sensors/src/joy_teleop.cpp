#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

#define PI 3.14159265

class TeleopJoy
{	
	private:
		geometry_msgs::Twist cmd;
		double max_vx, max_vz, vel_linear, vel_angular, step, step_turn;
		int step_forward, step_backward, step_turn_right, step_turn_left;	//botoes usados
		bool deadman;
		
		ros::NodeHandle node;
		ros::Publisher vel_pub;
		ros::Subscriber joy_sub;
		
	public:
		TeleopJoy()
		{
			max_vx = 1.0;
			max_vz = 1.0;
			vel_linear = 0.0;
			vel_angular = 0.0;
			step = 0.1;
			step_forward = 0;
			step_backward = 2;
			step_turn = 0.1;
			step_turn_left = 3;
			step_turn_right = 1;
		}
		
		~TeleopJoy() {}
		
		void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
		{
			deadman = (joy_msg->buttons[4] || joy_msg->buttons[5] || joy_msg->buttons[6] || joy_msg->buttons[7]);
			
    		if(deadman){
    			vel_linear = 0.0;
    			return;
    		}
    		    		
    		if(joy_msg->buttons[step_forward] && vel_linear < max_vx)
    			vel_linear = vel_linear + step;
		
		else if(joy_msg->buttons[step_backward] && vel_linear > - max_vx)
    			vel_linear = vel_linear - step;
    			
		if(joy_msg->buttons[step_turn_right] && vel_angular > - max_vz)
			vel_angular = vel_angular - step_turn;
		
		else if(joy_msg->buttons[step_turn_left] && vel_angular < max_vz)
			vel_angular = vel_angular + step_turn;
		
		}
		
		void send_cmd_vel()
		{
			cmd.linear.x = vel_linear;
			cmd.angular.z = vel_angular;
			vel_pub.publish(cmd);
			ROS_INFO("vel. linear = %.2f --- vel. angular = %.2f", vel_linear, vel_angular);
		}
		
		void init()
		{
		node.param("max_vx", max_vx, max_vx);
        	node.param("max_vz", max_vz, max_vz);
        	node.param("step_forward", step_forward, step_forward);
        	node.param("step_backward", step_backward, step_backward);
        	node.param("step", step, step);
		node.param("step_turn", step_turn, step_turn);
		node.param("step_turn_left", step_turn_left, step_turn_left);
		node.param("step_turn_right", step_turn_right, step_turn_right);
		
		ROS_DEBUG("max_vx: %.2f m/s\n", max_vx);
        	ROS_DEBUG("max_vz: %.2f deg/s\n", max_vz * 180.0 / PI);
        	ROS_DEBUG("go forward button: %d\n", step_forward);
        	ROS_DEBUG("go backward button: %d\n", step_backward);
        	ROS_DEBUG("turn left button: %d\n", step_turn_left);
        	ROS_DEBUG("turn right button: %d\n", step_turn_right);
		ROS_DEBUG("stop buttons are 1 and 2 Right and 1 and 2 Left\n");
			
		vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        	joy_sub = node.subscribe("joy", 1, &TeleopJoy::joy_callback, this);
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_joy");
	
	TeleopJoy *teleop = new TeleopJoy();
	
	teleop->init();
	
	ros::Rate rate(20);
	
	while(ros::ok()){
		ros::spinOnce();
		teleop->send_cmd_vel();
		rate.sleep();
	}
	
	return 0;
}
