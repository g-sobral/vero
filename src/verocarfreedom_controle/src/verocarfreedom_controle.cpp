#include "ros/ros.h"
#include "teleop_source_keyboard.hpp"
#include "teleop_source_joystick.hpp"
#include <verocarfreedomdefs_msgs/CarCommand.h>

#include "watchdog/watchdog.hpp"

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

using namespace teleop;
using namespace std;

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

void AtualizaComandoJoystick (TeleopState estado, verocarfreedomdefs_msgs::CarCommand &comando){
	// Calcula a aceleracao e a velocidade com base nos valores do joystick e dos limites do carro.
	comando.speedLeft = comando.speedRight = Sinal(estado.axes[1].value) * Min (Modulo(estado.axes[1].value * AccelSpeedMax(estado.axes[1].value) / CYCLE) + Modulo (comando.speedLeft), Modulo (estado.axes[1].value * VelMax (estado.axes[1].value)));
	comando.steerAngle = Sinal(-estado.axes[0].value) * Min (Modulo(estado.axes[0].value * AccelAngleMax (estado.axes[0].value) / CYCLE) + Modulo (comando.steerAngle), Modulo (estado.axes[0].value * AngleMax (estado.axes[0].value, comando.speedRight)));
} // AtualizaComandoJoystick

void AtualizaComandoKeyboard (TeleopState &estado, verocarfreedomdefs_msgs::CarCommand &comando){
	// Calcula a aceleracao e a velocidade com base nos valores do keyboard e dos limites do carro.
	comando.speedLeft = comando.speedRight = estado.axes[0].value * VelMax (estado.axes[1].value);
	comando.steerAngle = -AngleMax (estado.axes[1].value, comando.speedRight, true) * Modulo (estado.axes[1].value);
} // AtualizaComandoKeyboard

int main (int argc, char **argv){
	int opt;
	verocarfreedomdefs_msgs::CarCommand comando;
	TeleopState estado;
	bool updated, joystick = false;
	unsigned int listenTimeout = 1;
	// Inicia o modulo no ros.
	ros::init(argc, argv, "verocarfreedom_controle_node");
	ros::NodeHandle n;
   WatchDog* watchdog;
   ros::AsyncSpinner spinner(2);
   spinner.start();
   watchdog = new WatchDog(n);

	LimitsInit ();

	ros::Publisher controle_pub = n.advertise<verocarfreedomdefs_msgs::CarCommand>("car_command", 1);

  while ((opt = getopt(argc, argv, "j:")) != -1)
    {
      switch (opt)
        {
        case 'j':
			joystick = true;
	  break;
        default:
	  cout << "Usage: " << argv[0] << " [-j]" << endl << endl
	       << "-j j   \tRecebe comandos do joystick." << endl;
	  return 1;
        }
    }

  // Inicia o WatchDog
	double watchdog_duration;
	ros::NodeHandle nh("~");
   nh.param<double>("Carfreedom_controle_watchdog_duration", watchdog_duration, 2.0);
   watchdog->StartTimer(watchdog_duration);

	if (joystick){
		TeleopSourceJoystick *driverJoystick = new TeleopSourceJoystick ();
		driverJoystick->setDevice ("/dev/input/js0");
		driverJoystick->init();
		
		ros::Rate cycle(CYCLE);
		while (ros::ok()){
			ros::spinOnce();
			driverJoystick->listen(listenTimeout, &estado, &updated);
			AtualizaComandoJoystick (estado, comando);
			controle_pub.publish (comando);
			watchdog->IsAlive();
			cycle.sleep();
		}
	} else /* keyboard */{
		TeleopSourceKeyboard *driverKeyboard = new TeleopSourceKeyboard ();
		driverKeyboard->setSteps(CYCLE/accel_limits[0]); // define todas as acelerações como aproximadamente o valor da max_accel_speed_foward. O valor não pode ser definido precisamente porque o mesmo depende da velocidade de repetição de teclas do sistema e todas as teclas possuem a mesma aceleração. 
		driverKeyboard->init();

		ros::Rate cycle(CYCLE);
		while (ros::ok()){
			ros::spinOnce();
			driverKeyboard->listen(listenTimeout, &estado, &updated);
			if (updated) {
			AtualizaComandoKeyboard (estado, comando);
			controle_pub.publish (comando);
			}
			watchdog->IsAlive(); //The keyboard only send messages when updated, so the lack of publishments does not emply that the node is dead.
			cycle.sleep();
		}
	}
	return 0;
}
