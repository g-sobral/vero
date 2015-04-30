#include "ptu_classes.h"

/**
* \brief Pacote PTU46_driver cria um node que serve como um wrapper do driver disponibilizado da biblioteca mrpt para o Pan-Tilt C46-17
* para a interface do ROS.
* \par Entradas: O node recebe do tópico PTU46_cmd mensagens do tipo Twist. O Pan recebe velociades angulares na componente z da mensagem e o Tit
* na componente y. Para o Pan, uma velocidade positiva ofaz girar no sentido anti-horário e uma velocidade negativa, no sentido horário.
* \par Saída: O node publica sua orientação corrente no tópico PTU46_orientation mensagens do tipo Pose como um Quaternion e sua velocidade atual
* no tópico PTU46_velocity mensagens do tipo Twist, dado que a componente z é a velocidade do Pan e a componente y a velocidade do Tilt
* Ha tambem transformacoes de coordenadas utilizando o pacote TF e publicadas no topico /tf. A base do PanTilt, denominada PanTil_base é a referencia
* para as posteriores transformaçoes. A primeira transformaçao ocorre do PanTilt_base para o base_PanTilt_arm onde é realizada uma translação fixa
* decorrente da forma física do aparelho e uma translaçao dinamica circular decorrente do movimento Pan. Alem disso, uma rotaçao ao redor do eixo z
* tambem é realizada fazendo com que a componente x do frame base_PanTilt_arm esteja sempre tangenciando a trajetoria circular. Por ultimo, ocorre
* uma rotaçao ao redor do eixo y que é o Tilt do base_PanTilt_arm para o PanTilt_arm. O frame base_PanTilt_arm foi criado apenas por questoes operacionais
* e nao é efetivamente utilizado, assim os frames que realmente representam o PanTil é o PanTil_base e o PanTilt_arm  
*/

void ctrlHandler(int x)
{	
	Publisher p;
	string t;
	ptu_class *instance = ptu_class::uniqueInst(p, p, t);	
	
	instance->getptu()->halt('P');
	instance->getptu()->halt('T');
	instance->getptu()->close();
	sleep(1);
	exit(0);	
}

int main(int argc,char **argv)
{
	init(argc, argv, "PTU46_driver");
	NodeHandle node;
	
	ptu_class* ptu_obj;
	std::string port;
        node.param<std::string>("port", port, "/dev/ttyUSB1");

	ptu_obj = ptu_class::uniqueInst(node.advertise<geometry_msgs::Twist>("PTU46_velocity", 1), node.advertise<geometry_msgs::Pose>("PTU46_pose", 1), port);
	
	signal(SIGINT, ctrlHandler);
	signal(SIGABRT, ctrlHandler);

	Subscriber sub = node.subscribe("PTU_cmd_vel", 1, &ptu_class::PTU46_callback, ptu_obj);
	
	spin();
	
	return 0;	
}
