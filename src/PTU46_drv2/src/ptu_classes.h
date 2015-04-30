#include <math.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <signal.h>
#include "ros/ros.h"
#include <mrpt/hwdrivers/CPtuDPerception.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "kdl_parser/kdl_parser.hpp"

#define PI 3.14159265

using namespace ros;
using namespace KDL;
using namespace tf;
using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;

/*
* Padrão Singleton: evita a criação de múltiplos objetos e resolve o problema de se utilizar variáveis globais de forma mais elegante
*/

class ptu_class{
	private:
		static ptu_class* instance;
		Publisher *vel_pub;
		Publisher *pose_pub;
		CPtuDPerception *ptu;
		TransformBroadcaster br;
   		Transform transform;
		double v_angularP;
		double v_angularT;
		double max_posP;
		double max_posT;
		double pos_atualP;
		double pos_atualT;
	
	protected:
		ptu_class(Publisher p1, Publisher p2, string port);
	
	public:
		static ptu_class* uniqueInst(Publisher p1, Publisher p2, string port);
		void PTU46_callback(const geometry_msgs::Twist &msg);
		CPtuDPerception* getptu();

};

