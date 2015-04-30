#include "ros/ros.h"
#include "candriver.h"

void escreveMensagem (socketcan_lib::CANDriver driver){
	struct can_frame frame;
	frame.can_id = 7;
	frame.can_dlc = 2;
	frame.data[0] = 128;
	frame.data[1] = 123;

	driver.writeCanFrame(frame);
}

void leMensagem (socketcan_lib::CANDriver driver){
	unsigned int nframes = 1;
	driver.readData (nframes);
}

int main (int argc, char **argv){
	int opt;
	socketcan_lib::Context prop;
	bool verbose = false;
  // Inicia o modulo no ros.
  ros::init(argc, argv, "socketcan_lib_test_node");
  ros::NodeHandle n;
	socketcan_lib::IniciaContext (prop);
	  // Caso nescessário a porta padrão pode ser configurada pela linha de comando.
  while ((opt = getopt(argc, argv, "i:t:d:b:v:")) != -1)
    {
      switch ( opt )
        {
        case 'i':			
				prop.interfaceCan = optarg;
          break;
        case 't':			
				prop.timeOut = atoi(optarg);
          break;
        case 'd':			
				prop.devCan = optarg;
          break;
        case 'b':			
				prop.baudRate = atoi(optarg);
        case 'v':			
				verbose = atoi(optarg);
          break;
        default:
          std::cout << "Usage: " << argv[0] << " [-i interfaceCan -t timeOut -d devCan -b baudRate -v 0]" << std::endl;
          return 1;
        }
    }

	 socketcan_lib::CANDriver driver(prop, verbose);
	ROS_INFO("driver iniciado!");

	// Escreve 5 mensagens por segundo.
	ros::Rate cycle(5);
	// Escreve e le mensagens.
	while (ros::ok()) {
	ros::spinOnce();
	ROS_INFO("escrevendo...");
	escreveMensagem (driver);
	ROS_INFO("lendo...");
	leMensagem (driver);
	}
	return 0;
}
