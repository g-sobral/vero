#include "ros/ros.h"
#include "candriver.h"

void leMensagem (socketcan_lib::CANDriver driver){
	unsigned int nframes = 1;
	driver.readData (nframes);
}

int main (int argc, char **argv){
	int opt;
	socketcan_lib::Context prop;
	bool verbose = false;
  // Inicia o modulo no ros.
  ros::init(argc, argv, "socketcan_lib_test_leitura");
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
	verbose = true;
	// Inicia o driver.
	 socketcan_lib::CANDriver driver(prop, verbose);
	ROS_INFO("driver iniciado!");
	// Le as mensagens.
	while (ros::ok()) {
	ros::spinOnce();
	ROS_INFO("lendo...");
	leMensagem (driver);
	}
	return 0;
}
