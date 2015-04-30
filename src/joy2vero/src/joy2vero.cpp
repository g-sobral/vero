#include <ros/ros.h>
#include <verocarfreedomdefs_msgs/CarCommand.h>
	
#include <sensor_msgs/Joy.h>

#define CYCLE 200
#define PERIODO 2
#define FAIXA_MAX_SPEED 0.8 // A partir de quantos % da velocidade máxima o steerAngle deve ser limitado

#define CARFREEDOM_MAX_SPEED_FORWARD_M_S 10
#define CARFREEDOM_MAX_SPEED_REVERSE_M_S 10
#define CARFREEDOM_MAX_STEERING_ANGLE_DEG 25
#define CARFREEDOM_MAX_STEERING_ANGLE_AT_MAX_SPEED_DEG 20

#define CARFREEDOM_MAX_JOYSTICK_FORWARD 1023
#define CARFREEDOM_MAX_JOYSTICK_REVERSE 0
#define CARFREEDOM_MAX_JOYSTICK_LEFT 0
#define CARFREEDOM_MAX_JOYSTICK_RIGHT 1023




double speed_limits[2];//{max_speed_foward, max_speed_reverse};
double angle_limits[4];//{max_steerAngle_right, max_steerAngle_left, max_steerAngle_right_at_max_speed, max_steerAngle_left_at_max_speed};
double accel_limits[4];//{max_accel_speed_foward, max_accel_speed_reverse, max_accel_steerAngle_right, max_accel_steerAngle_left};


using namespace std;

class Joy2Vero
{
public:
  Joy2Vero();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void AtualizaComandoJoystick (double raw_linear, double raw_angle);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  verocarfreedomdefs_msgs::CarCommand command;
  ros::Publisher command_pub_;
  ros::Subscriber joy_sub_;
  
};



int Sinal (double a){
	if (a < 0) return -1;
	else return 1;
}// Sinal

double Modulo (double a){
	if (a < 0) return -a;
	else return a;
}// Modulo

double Min (double a, double b){
	if (a < b) return a;
	else return b;
}// Min

bool AtMaxSpeed (double current_speed){
	if (current_speed > 0 && current_speed >= FAIXA_MAX_SPEED * speed_limits[0]) return true;
	else if (current_speed < 0 && current_speed <= -FAIXA_MAX_SPEED * speed_limits[1]) return true;
	return false;
}// AtMaxSpeed

double VelMax (double joystick_speed){
	// Calcula a velocidade máxima permitida dependendo da orientação.
	if (joystick_speed > 0) return speed_limits[0];
	else return speed_limits[1];
}// VelMax

double AngleMax (double &joystick_angle, double current_speed, bool keyboard = false){
	// Calcula o steerAngle maximo permitido dependendo da orientação e da velocidade.
	int incremento = 0;
	bool corrigeAngle = false;
	if (AtMaxSpeed(current_speed)) {
		if (!keyboard)incremento = 2;
		corrigeAngle = true; // No caso do keyboard é necessária uma correção do angulo para evitar saltos.
	}

	if (joystick_angle > 0) {
		if (keyboard && corrigeAngle)joystick_angle = Sinal (joystick_angle) * Min (Modulo(angle_limits[2] / angle_limits[0]), Modulo(joystick_angle));
		return angle_limits[0+incremento];
		}
	else {
		if (keyboard && corrigeAngle)joystick_angle = Sinal (joystick_angle) * Min (Modulo(angle_limits[3] / angle_limits[1]), Modulo(joystick_angle));
		return angle_limits[1+incremento];
	}
}// AngleMax

double AccelSpeedMax (double joystick_speed){
	// Calcula a aceleração máxima permitida dependendo da orientação.
	if (joystick_speed > 0) return accel_limits[0];
	else return accel_limits[1];
}// AccelSpeedMax

double AccelAngleMax (double joystick_angle){
	// Calcula a aceleracao maxima do steerAngle dependendo da orientação.
	if (joystick_angle > 0) return accel_limits[2];
	else return accel_limits[3];
}// AccelAngleMax

void LimitsInit (){
	ros::NodeHandle nh("~");
	// Init speed limits.
   nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxForwardSpeed", speed_limits[0], CARFREEDOM_MAX_SPEED_FORWARD_M_S);
	nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxReverseSpeed", speed_limits[1], CARFREEDOM_MAX_SPEED_REVERSE_M_S);
	// Init angle limits.
   nh.param<double>("CarFreedom_Config_CarFreedom_MaxSteerAngleRight", angle_limits[0], CARFREEDOM_MAX_STEERING_ANGLE_DEG);
   nh.param<double>("CarFreedom_Config_CarFreedom_MaxSteerAngleLeft", angle_limits[1], CARFREEDOM_MAX_STEERING_ANGLE_DEG);
   nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxSteerAngleAtMaxSpeed", angle_limits[2], CARFREEDOM_MAX_STEERING_ANGLE_AT_MAX_SPEED_DEG);
   nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxSteerAngleAtMaxSpeed", angle_limits[3], CARFREEDOM_MAX_STEERING_ANGLE_AT_MAX_SPEED_DEG);
	// Init accel limits
	nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxFowardAcceleration", accel_limits[0], 4.0);
	nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxReverseAcceleration", accel_limits[1], 4.0);
	nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxSteerAngleRightAcceleration", accel_limits[2], 4.0);
	nh.param<double>("CarFreedom_Config_VehicleDescription_Control_VelocityBicycle_MaxSteerAngleLeftAcceleration", accel_limits[3], 4.0);
} // LimitsInit

void Joy2Vero::AtualizaComandoJoystick (double raw_linear, double raw_angle){
	// Calcula a aceleracao e a velocidade com base nos valores do joystick e dos limites do carro.
	ROS_DEBUG_STREAM("SpeedLeft Calculus: " << Modulo(raw_linear * AccelSpeedMax(raw_linear) / CYCLE) + Modulo (command.speedLeft));
	ROS_DEBUG_STREAM("SpeedLeft MAX: " << (raw_linear * VelMax (raw_linear)));
	command.speedLeft = command.speedRight = 
		Sinal(raw_linear) * Min (
			Modulo(raw_linear * AccelSpeedMax(raw_linear) / CYCLE) + Modulo (command.speedLeft), 
			Modulo (raw_linear * VelMax (raw_linear)));

	ROS_DEBUG_STREAM("Steer Angle Calculus: " << Modulo(raw_angle * AccelAngleMax (raw_angle) / CYCLE) + Modulo (command.steerAngle) );
	ROS_DEBUG_STREAM("Steer Angle MAX: " << (raw_angle * AngleMax (raw_angle, command.speedRight)));
	command.steerAngle = 
		Sinal(-raw_angle) * Min (
			Modulo(raw_angle * AccelAngleMax (raw_angle) / CYCLE) + Modulo (command.steerAngle), 				Modulo(raw_angle * AngleMax(raw_angle, command.speedRight)));
	ROS_DEBUG_STREAM("Speed Left: " << command.speedLeft);
	ROS_DEBUG_STREAM("Speed Right: " << command.speedRight);
	ROS_DEBUG_STREAM("Steer Angle: " << command.steerAngle);
} // AtualizaComandoJoystick


Joy2Vero::Joy2Vero():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  // Alterar para publicar mensagens do tipo CarCommand
 //vel_pub_ = nh_.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);
 command_pub_ = nh_.advertise<verocarfreedomdefs_msgs::CarCommand>("car_command", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Joy2Vero::joyCallback, this);

}

void Joy2Vero::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
// Declara mensagens do tipo CarCommand e preencher com os ados do leitor


  ROS_INFO_STREAM("\nLinear: " << joy->axes[linear_] << "\nAngular: " << joy->axes[angular_]);
  AtualizaComandoJoystick(l_scale_*joy->axes[linear_],a_scale_*joy->axes[angular_]);
  command_pub_.publish(command);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy2vero");
  Joy2Vero joy2vero;
  LimitsInit();

  ros::spin();
}
