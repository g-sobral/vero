#include "ros/ros.h"
#include "candriver.h"

void escreveMensagem (socketcan_lib::CANDriver driver){
	struct can_frame frame;
	frame.can_id = 0x100;
	frame.can_dlc = 2;
	frame.data[0] = 128;
	frame.data[1] = 123;

	driver.writeCanFrame(frame);
}

int main (int argc, char **argv){
	int opt;
	socketcan_lib::Context prop;
	bool verbose = false;
  // Inicia o modulo no ros.
  ros::init(argc, argv, "socketcan_lib_test_escrita");
  ros::NodeHandle n;
	socketcan_lib::IniciaContext (prop);
	   // Caso nescessário os paramatros podem ser configurados pela linha de comando.
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
	// Inicia o driver.
	 socketcan_lib::CANDriver driver(prop, verbose);

	ROS_INFO("driver iniciado!");
	// Escreve 5 mensagens por segundo.
	ros::Rate cycle(5);

	while (ros::ok()) {
	ros::spinOnce();
	ROS_INFO("escrevendo...");
	escreveMensagem (driver);

	cycle.sleep();
	}
	return 0;
}
