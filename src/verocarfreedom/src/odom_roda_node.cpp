#include "odom_roda_node.h"

#define PERIODO_PUBLISH_CAR_DATA 20000
#define PERIODO_PUBLISH_ODOM 20000
#define DISTANCIA_ENTRE_EIXOS 1.0

verocarfreedom::Driver *driver;
float k1, k2;

  tf::Quaternion q;
void callback (verocarfreedomdefs_msgs::CarCommand comando){
	driver->write(comando);
}

void cmd_vel_callback (const geometry_msgs::Twist& msg){
	verocarfreedomdefs_msgs::CarCommand comando;
	float velLinear = msg.linear.x, velAngular = msg.angular.z;
	comando.speedLeft = comando.speedRight = velLinear;
	comando.steerAngle = atan(velAngular * DISTANCIA_ENTRE_EIXOS / velLinear);

	driver->write(comando);
}
float min(float x1, float x2){
	if (x1<x2) return x1;
	return x2;
}

void AtualizaOdometria (nav_msgs::Odometry &odom, verocarfreedomdefs_msgs::CarData data){
	odom.header.stamp = data.timeStamp;

	odom.pose.pose.position.x = data.x;
	odom.pose.pose.position.y = data.y;
	odom.pose.pose.position.z = 0;
	// translate roll, pitch and yaw into a Quaternion
  geometry_msgs::Quaternion odom_quat;
  q.setRPY(0, 0, data.yaw); //roll, pitch, yaw

  tf::quaternionTFToMsg(q, odom_quat);	
	ROS_INFO_STREAM("data.vlong" << data.vlong);
	ROS_INFO_STREAM("data.vyaw" << data.dyaw);
	odom.pose.covariance = {0.001,   0.0,   0.0,   0.0,   0.0,   0.0, 
					0.0, 0.001,   0.0,   0.0,   0.0,   0.0,
					0.0,   0.0,  9999,   0.0,   0.0,   0.0, 
					0.0,   0.0,   0.0,  9999,   0.0,   0.0,
					0.0,   0.0,   0.0,   0.0,  9999,   0.0,
					0.0,   0.0,   0.0,   0.0,   0.0,   0.001};
 

	odom.pose.pose.orientation = odom_quat;

	odom.twist.twist.linear.x = data.vlong*cos(data.yaw);
	odom.twist.twist.linear.y = data.vlong*sin(data.yaw);
	odom.twist.twist.linear.z = 0;

	odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0;
 	odom.twist.twist.angular.z=data.dyaw;
	odom.twist.covariance = {0.001,   0.0,   0.0,   0.0,   0.0,   0.0, 
					0.0, 0.001,   0.0,   0.0,   0.0,   0.0,
					0.0,   0.0,  9999,   0.0,   0.0,   0.0, 
					0.0,   0.0,   0.0,  9999,   0.0,   0.0,
					0.0,   0.0,   0.0,   0.0,  9999,   0.0,
					0.0,   0.0,   0.0,   0.0,   0.0,   0.001};
	//ROS_INFO_STREAM(odom.twist.covariance);
} //AtualizaOdometria

//Publica a transform odom->base_link
// e a odometria
void AtualizaTransformEOdometria (geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom, verocarfreedomdefs_msgs::CarData data, bool invert){
		AtualizaOdometria(odom,	data);
		odom_trans.header.stamp = data.timeStamp;
		//define o frame_id e o child_frame_id no laco principal, fora dessa funcao
		//tf publicado pode ser invertida, util para o caso de outro odom ser publicado
		tf::Transform temp = tf::Transform(tf::createQuaternionFromYaw(data.yaw), tf::Vector3(data.x, data.y, 0));
		if(invert){
			odom_trans.header.frame_id=odom.child_frame_id;
			odom_trans.child_frame_id=odom.header.frame_id;
			temp=temp.inverse();

			
			
		}
		else{
			odom_trans.header.frame_id=odom.header.frame_id;
			odom_trans.child_frame_id=odom.child_frame_id;
		}
		tf::transformTFToMsg(temp, odom_trans.transform);
}
		
bool ComparaTimeStamp (ros::Time last, ros::Time current, int periodo){
	bool atualizar = false;
	if (current.sec > last.sec) atualizar = true;
	else if ((current.sec == last.sec) && ((current.nsec - periodo) >= last.nsec)) atualizar = true;
	return atualizar;
} //ComparaTimeStamp

int main (int argc, char **argv){
	int node_cycle;
	verocarfreedom::Context car_context;
	socketcan_lib::Context socket_context;
	verocarfreedomdefs_msgs::CarData data;
	nav_msgs::Odometry odom;
	geometry_msgs::TransformStamped odom_trans;
	ros::Time last_time_stamp_car_data, last_time_stamp_odom;
	bool publish_tf_, invert_tf_;

  // Inicia o modulo no ros.
  ros::init(argc, argv, "verocarfreedom_test_node");
  ros::NodeHandle n;
  WatchDog* watchdog;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  watchdog = new WatchDog(n);

	verocarfreedom::IniciaContext (car_context);
	socketcan_lib::IniciaContext (socket_context);
	
	 driver = new verocarfreedom::Driver (car_context, socket_context);

	ros::Publisher car_data_pub = n.advertise<verocarfreedomdefs_msgs::CarData>("car_data", 1);
	ros::Publisher odom_roda_pub = n.advertise<nav_msgs::Odometry>("roda_odom", 1);
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber comando_sub = n.subscribe("car_command", 1, callback);
	ros::Subscriber comando_sub_navigation = n.subscribe("cmd_vel", 1, cmd_vel_callback);

	//Parametros importantes para a mensagem de odometria
	ros::param::param<std::string>("~frame_id", odom.header.frame_id, "odom");
  	ros::param::param<std::string>("~child_frame_id", odom.child_frame_id, "base_link");
	ros::param::param<bool>("~publish_tf", publish_tf_,"false");
	ros::param::param<bool>("~invert_tf", invert_tf_,"false");
	ros::param::param<float>("~k1", k1, 0.05);
	ros::param::param<float>("~k2", k2, 0.05);
	//ROS_INFO_STREAM(odom.header.frame_id);

	last_time_stamp_odom = last_time_stamp_car_data = ros::Time::now();

	ros::NodeHandle nh("~");
   nh.param<int>("CarFreedom_Config_Odom_Roda_Node_Cycle", node_cycle, 100);
	ros::Rate cycle(node_cycle);

  // Inicia o WatchDog
	double watchdog_duration;
   nh.param<double>("CarFreedom_watchdog_duration", watchdog_duration, 2.0);
   watchdog->StartTimer(watchdog_duration);

	driver->enable();

	while(ros::ok()) {
		driver->read (data);
		if (ComparaTimeStamp (last_time_stamp_odom, data.timeStamp, PERIODO_PUBLISH_ODOM)){
			if(publish_tf_){
				AtualizaTransformEOdometria (odom_trans, odom, data, invert_tf_);
				odom_broadcaster.sendTransform(odom_trans);
				
			}
			else{
				AtualizaOdometria(odom, data);			
			}			
			
			odom_roda_pub.publish(odom);
			//odom_broadcaster.sendTransform(odom_trans);
			watchdog->IsAlive();
			last_time_stamp_odom = odom.header.stamp;
         driver->requery_now();
		}
		if (ComparaTimeStamp (last_time_stamp_car_data, data.timeStamp, PERIODO_PUBLISH_CAR_DATA)){
			car_data_pub.publish(data);
			watchdog->IsAlive();
			last_time_stamp_car_data = data.timeStamp;
		}
		cycle.sleep();
		ros::spinOnce();
	}

	return 0;
}
