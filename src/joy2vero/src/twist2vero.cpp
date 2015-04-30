#include <ros/ros.h>
#include <verocarfreedomdefs_msgs/CarCommand.h>
#include <geometry_msgs/Twist.h>


class Twist2Vero
{
public:
  Joy2Vero();

private:
  void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Subscriber twist_sub_;
  
};



Twist2Vero::Twist2Vero()
{

  command_pub_ = n.advertise<verocarfreedomdefs_msgs::CarCommand>("car_command", 1);
  twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Twist2Vero::twistCallback, this);

}

void Twist2Vero::twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
// Declara mensagens do tipo CarCommand e preencher com os ados do leitor

  verocarfreedomdefs_msgs::CarCommand command


//TODO implementar calculo usado em verocarfreedom_controle
  command.steerAngle = twist->axes[angular_];
  command.speedLeft = command.speedRight= twist->linear.x;
  command_pub_.publish(command);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}
