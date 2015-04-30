#include "sockutil/sockclient.h"

int main(int argc, char **argv) {

sockutil::StaubliClient s("localhost",7);//echo server 
int go=5; // NUMBER OF ATTEMPTS
while(go--) {
  ros::init(argc, argv, "testsocknode");
  ros::NodeHandle nh;
  boost::array<double,6> msg;
  msg[0] = 0.1;msg[1] = 1.2;msg[2] = 2.3;msg[3] = 3.4;msg[4] = 4.5;msg[5] = 5.6;
  s.connect();
  //std::string msgsend = "1.1 2.2 3.3 4.4 5.5 6.6 7.7 8.8 9.9 10.1 11.2 12.3\n";
  s.send(msg);
  
  boost::array<double,6> res;
    boost::array<double,6> res2;
  s.rec(res,res2);
  
  std::cerr << "got \n" << res[0] << " " << res[1] << " "<< res[2] << " "<< res[3] << " "<< res[4] << " " << res[5] ;
  std::cerr << "\n" << res2[0] << " " << res2[1] << " "<< res2[2] << " "<< res2[3] << " "<< res2[4] << " " << res2[5] ;

  usleep(1000 * 4);
}

}