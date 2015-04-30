#include "ransac_lib.hpp"
#include "control.hpp"
#include "ransac_classes.hpp"

/*
*	parametros:
*	v_linear: define a velocidade linear do VERO
*	lenght: distancia entre eixos do VERO
	which_car: define para que carro o controle sera efetuado
*	parametros do controlador PID:
*	KPT:
*	KIT:
*	KRT:
*	KVT:
*
*	ransacCallback:funçao que recebe duas linhas e cria uma bissetriz para que a funçao de Controle possa aplicar
*	ao movimento correto
*	Entrada: mensagem ransac_project::BorderLines que define as linhas esquerda e direita e mensagem nav_msgs::Odometry
*	para obter-se a velocidade angular atual do Pioneer
*	Saidas: publica a mensagen ransac_project::CarCommand, que envia o comando para o VERO sendo ele uma velocidade linear
*	e um angulo de rotaçao ou a mensagem geometry_msgs::Twist que envia o comando para o Pioneer sendo ele uma velocidade linear e uma angular. Há *	a mensagem ransac_project::Bisectrix, que envia a reta bissetriz encontrada a partir das retas
*	laterais. Esta mensagem é apenas para visualização do processo
*/

void ctrlHandler(int x)
{	
	NodeHandle n;
	Publisher p2;
	ransacControl *rc2 = ransacControl::uniqueInst(p2, false, n);
	
	if(rc2->getwhich_car().compare("vero") == 0){
	
		ransac_project::CarCommand msg_vero;
	
		msg_vero.speedLeft = 0;
		msg_vero.speedRight = 0;
		msg_vero.steerAngle = 0;
	
		rc2->publica(msg_vero);
	}
	
	else{
		
		geometry_msgs::Twist msg_pioneer;
	
		msg_pioneer.linear.x = 0;
		msg_pioneer.linear.y = 0;
		msg_pioneer.linear.z = 0;
		msg_pioneer.angular.x = 0;
		msg_pioneer.angular.y = 0;
		msg_pioneer.angular.z = 0;
		
		rc2->publica(msg_pioneer);
	}
	
	sleep(1);
	exit(0);
}

void ransacControl::odometryCallback(const nav_msgs::Odometry &Odom_msg)
{
	angularVel = Odom_msg.twist.twist.angular.z;
}

void ransacControl::ransacCallback(const ransac_project::Bisectrix &biMsg)
{
	watchdog->IsAlive();	
	
	double rudder;
	vector<double> bisectrix(4);

	bisectrix[0] = -15;
	bisectrix[1] = 15;
	bisectrix[2] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[0])/biMsg.bisectrix[1];
	bisectrix[3] = -(biMsg.bisectrix[2] + biMsg.bisectrix[0]*bisectrix[1])/biMsg.bisectrix[1];
	
	rudder = Controle::LineTracking(bisectrix, v_linear, angularVel, dt, KPT, KIT, KRT, KVT);	/*funçao de controle*/
	dt = ros::Time::now().toSec();
	
	if(which_car.compare("vero") == 0){
		
		ransac_project::CarCommand msg_vero;
	
		msg_vero.speedLeft = v_linear;		/*speedLeft = speedRight*/
		msg_vero.speedRight = v_linear;
		msg_vero.steerAngle = atan(rudder * lenght / v_linear);
	
		ROS_INFO("steerAngle: (%.2f)", msg_vero.steerAngle);
	
		pub->publish(msg_vero);
	}
	
	else{
		
		geometry_msgs::Twist msg_pioneer;
	
		msg_pioneer.linear.x = v_linear;	/*speedLeft = speedRight*/
		msg_pioneer.linear.y = 0;
		msg_pioneer.linear.z = 0;
		msg_pioneer.angular.x = 0;
		msg_pioneer.angular.y = 0;
		msg_pioneer.angular.z = rudder;
		
		//ROS_INFO("rudder: (%.2f)", msg_pioneer.angular.z);
		
		pub->publish(msg_pioneer);
	}
}

int main(int argc, char **argv)
{
	init(argc, argv, "ransac_control");
	NodeHandle node;
	NodeHandle node_aux("~");
	
	ransacControl* rc;
	string which_car;
	bool wp = false;
	double v_linear, KPT, KIT, KRT, KVT, lenght = 0;
	
	if(!node_aux.getParam("which_car", which_car)){
		
		ROS_ERROR("must specify vero or pioneer");
		exit(0);
	}
	
	if(which_car.compare("vero") == 0){
		
		if(!node_aux.getParam("v_linear", v_linear) || !node_aux.getParam("KPT", KPT) || !node_aux.getParam("KIT", KIT) ||!node_aux.getParam("KRT", KRT) ||!node_aux.getParam("KVT", KVT) || !node_aux.getParam("lenght", lenght)){
		
		ROS_ERROR("parameters not specified");
		exit(0);
		}
		
		rc = ransacControl::uniqueInst(node.advertise<ransac_project::CarCommand>("car_command", 1), wp, node);
	}

	else if(which_car.compare("pioneer") == 0){
		
		if(!node_aux.getParam("v_linear", v_linear) || !node_aux.getParam("KPT", KPT) || !node_aux.getParam("KIT", KIT) ||!node_aux.getParam("KRT", KRT) ||!node_aux.getParam("KVT", KVT)){
		
		ROS_ERROR("parameters not specified");
		exit(0);
		}
		
		rc = ransacControl::uniqueInst(node.advertise<geometry_msgs::Twist>("cmd_vel", 1), wp, node);
	}
	
	else{
		
		ROS_ERROR("must specify vero or pioneer");
		exit(0);
	}
	
	/*inicialização dos parametros fundamentais ao controle do Pioneer pelo RANSAC*/
	rc->setwhich_car(which_car);
	rc->setangularVel(0);
	rc->setdt(0);
	rc->setv_linear(v_linear);
	rc->setKPT(KPT);
	rc->setKIT(KIT);
	rc->setKRT(KRT);
	rc->setKVT(KVT);
	rc->setlenght(lenght);
	
	signal(SIGINT, ctrlHandler);
	signal(SIGABRT, ctrlHandler);
	
	Subscriber subOdom = node.subscribe("pose", 1, &ransacControl::odometryCallback, rc);
	Subscriber sub = node.subscribe("bisec_data", 1, &ransacControl::ransacCallback, rc);
	spin();
	
	return 0;
}
