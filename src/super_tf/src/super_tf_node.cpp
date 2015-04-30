/*
 * Controla a arvore de TF do carro, enviando todas as transformacoes
 * de todos os sensores e garantindo que a arvore esteja sempre completa.
 */

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <verocarfreedomdefs_msgs/CarData.h>

#define SUPER_TF_NODE_NAME "super_tf_node"

/////////////////////////// Variaveis Globais /////////////////////////

// Posicoes variaveis.
geometry_msgs::PoseWithCovariance odomPose, gpsPose, fusionPose, frontWheelPose;
// Posicoes constantes.
geometry_msgs::PoseWithCovariance firstOdom, gpsAntenaPose, xsensPose, xsensGpsAntenaPose, laserHokuyoPose, laserSick1Pose,  laserSick2Pose, cameraViderePose, imuVG400Pose, footprintPose;
// True caso firstOdom tenha sido recebido.
bool firstFix, pubMapOdom, pubOdomVehicle;
// Transforms names
std::string utmTransformName, mapTransformName, odomTransformName, vehicleTransformName, hokuyoTransformName, sick1TransformName, xsensTransformName, xsensAntenaGpsTransformName, sick2TransformName, videreTransformName, antenaGpsTransformName, imuTransformName, footprintTransformName, frontWheelTransformName;


///////////////////////// Funcoes Auxiliares /////////////////////////////

void PublishTF(const geometry_msgs::PoseWithCovariance& msg, std::string parent_name, std::string child_name){
	// Envia uma transformacao com base na posicao e na origem do sistema de coordenadas (parent_name).
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion quat;
	tf::quaternionMsgToTF (msg.pose.orientation, quat);

	transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
	transform.setRotation(quat);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_name, child_name));
	return;
} // PublishTF

void CopyOdomTo (const geometry_msgs::PoseWithCovariance& fromMsg,
						 geometry_msgs::PoseWithCovariance& toMsg){
	// Copia uma posicao em outra.
	toMsg.pose.position.x    = fromMsg.pose.position.x;
	toMsg.pose.position.y    = fromMsg.pose.position.y;
	toMsg.pose.position.z    = fromMsg.pose.position.z;
	toMsg.pose.orientation.x = fromMsg.pose.orientation.x;
	toMsg.pose.orientation.y = fromMsg.pose.orientation.y;
	toMsg.pose.orientation.z = fromMsg.pose.orientation.z;
	toMsg.pose.orientation.w = fromMsg.pose.orientation.w;
	return;
} // CopyOdomTo

geometry_msgs::PoseWithCovariance CalculateRelativePose (const geometry_msgs::PoseWithCovariance& origin,
									 									const geometry_msgs::PoseWithCovariance& point){
	// Calcula uma posicao em relacao a outra, utilizando a arvore de transformacoes gerada por este node.
	geometry_msgs::PoseWithCovariance relativePose;

	relativePose.pose.position.x = point.pose.position.x - origin.pose.position.x;
	relativePose.pose.position.y = point.pose.position.y - origin.pose.position.y;
	relativePose.pose.position.z = point.pose.position.z - origin.pose.position.z;

	tf::Quaternion quatOrigin, quatPoint;
	tf::quaternionMsgToTF (origin.pose.orientation, quatOrigin);
	tf::quaternionMsgToTF (point.pose.orientation, quatPoint);
	tf::quaternionTFToMsg (quatPoint * (quatOrigin.inverse()), relativePose.pose.orientation);
	return relativePose;
} // CalculateRelativePose

void IniciaPoseWithCovariance (geometry_msgs::PoseWithCovariance& msg){
	// Inicia uma mensagem PoseWithCovariance em branco.
	msg.pose.position.x = msg.pose.position.y = msg.pose.position.z = 0.0;
	tf::quaternionTFToMsg (tf::createIdentityQuaternion(), msg.pose.orientation);
	return;
} // IniciaPoseWithCovariance

bool validPose (const geometry_msgs::PoseWithCovariance& msg){
	return !(msg.pose.orientation.x==0 && msg.pose.orientation.y==0 && msg.pose.orientation.z==0 && msg.pose.orientation.w==0);
}

/////////////////////////////// Callbacks /////////////////////////


void SendFrontWheelTF( verocarfreedomdefs_msgs::CarData data){
	geometry_msgs::PoseWithCovariance pose;
	IniciaPoseWithCovariance(pose);
	ros::param::param<double>("FrontWheel_X", pose.pose.position.x, 0.0);
	ros::param::param<double>("FrontWheel_Y", pose.pose.position.y, 0.0);
	ros::param::param<double>("FrontWheel_Z", pose.pose.position.z, 0.0);
	pose.pose.orientation= 	tf::createQuaternionMsgFromYaw(data.steerAngle);
	PublishTF(pose, vehicleTransformName, frontWheelTransformName);

}
void SendOdom (const nav_msgs::Odometry& msg){
	// Atualiza a posicao armazenada na memoria e envia a transformacao
	if (validPose(msg.pose)){
		CopyOdomTo (msg.pose, odomPose);
		PublishTF (odomPose, odomTransformName, vehicleTransformName);
	}
	return;
} // SendOdom

void SendOdomMapCorrection (const nav_msgs::Odometry& msg){
	// Atualiza a posicao armazenada na memoria e envia a transformacao
	if (validPose(msg.pose)){
		if (firstFix) {
				firstFix = 0;
				CopyOdomTo (msg.pose, firstOdom);
		}	
		CopyOdomTo (msg.pose, fusionPose);
		PublishTF (CalculateRelativePose(odomPose, CalculateRelativePose(firstOdom, fusionPose)), mapTransformName, odomTransformName);
	}
	return;
} // SendOdomMapCorrection

void SendOdomMapCorrection (const nav_msgs::Odometry& msg, bool updateFirstOdom){
	// Versao que nunca atualiza o fix. E utilizada para reenviar o TF do gps sem perder o firstOdom
	bool aux=firstFix;
	firstFix = 0;
	SendOdomMapCorrection (msg);
	firstFix = aux;
} // SendOdomGpsTF

////////////////////////////////////////////////////////////////////

void IniciaTransformsNames (){
	// Inicia os nomes das transformacoes.
	ros::NodeHandle nh("~");
   nh.param<std::string>("UTMTransformName", utmTransformName, "UTM");
   nh.param<std::string>("MapTransformName", mapTransformName, "map");
   nh.param<std::string>("OdomTransformName", odomTransformName, "odom");
   nh.param<std::string>("VehicleTransformName", vehicleTransformName, "base_link");
   nh.param<std::string>("HokuyoTransformName", hokuyoTransformName, "hokuyo_laser");
   nh.param<std::string>("Sick1TransformName", sick1TransformName, "sick1_laser");
   nh.param<std::string>("Sick2TransformName", sick2TransformName, "sick2_laser");
   nh.param<std::string>("VidereTransformName", videreTransformName, "camera_videre");
   nh.param<std::string>("AntenaGpsTransformName", antenaGpsTransformName, "antena_gps");
   nh.param<std::string>("IMUTransformName", imuTransformName, "imu_vg400");
   nh.param<std::string>("xsensTransformName", xsensTransformName, "xsens");
   nh.param<std::string>("xsensAntenaGpsTransformName", xsensAntenaGpsTransformName, "xsens_gps");
   nh.param<std::string>("footprintTransformName", footprintTransformName, "base_footprint");
   nh.param<std::string>("frontWheelTransformName", frontWheelTransformName, "front_wheel");
}

void IniciaConstantPoses (){
	// Inicia as posições dos sensores fixos.
	ros::NodeHandle nh("~");
	// Laser Hokuyo
   nh.param<double>("HokuyoPose_x", laserHokuyoPose.pose.position.x, 0.0);
   nh.param<double>("HokuyoPose_y", laserHokuyoPose.pose.position.y, 1.805);
   nh.param<double>("HokuyoPose_z", laserHokuyoPose.pose.position.z, 0.18);
   nh.param<double>("HokuyoOrientation_x", laserHokuyoPose.pose.orientation.x, 0.0);
   nh.param<double>("HokuyoOrientation_y", laserHokuyoPose.pose.orientation.y, 0.0);
   nh.param<double>("HokuyoOrientation_z", laserHokuyoPose.pose.orientation.z, 0.0);
   nh.param<double>("HokuyoOrientation_w", laserHokuyoPose.pose.orientation.w, 1.0);
	// Laser SICK1
   nh.param<double>("Sick1Pose_x", laserSick1Pose.pose.position.x, 0.0);
   nh.param<double>("Sick1Pose_y", laserSick1Pose.pose.position.y, 1.755);
   nh.param<double>("Sick1Pose_z", laserSick1Pose.pose.position.z, 0.477);
   nh.param<double>("Sick1Orientation_x", laserSick1Pose.pose.orientation.x, 0.0);
   nh.param<double>("Sick1Orientation_y", laserSick1Pose.pose.orientation.y, 0.0);
   nh.param<double>("Sick1Orientation_z", laserSick1Pose.pose.orientation.z, 0.0);
   nh.param<double>("Sick1Orientation_w", laserSick1Pose.pose.orientation.w, 1.0);
	// Laser SICK2
   nh.param<double>("Sick2Pose_x", laserSick2Pose.pose.position.x, 0.0);
   nh.param<double>("Sick2Pose_y", laserSick2Pose.pose.position.y, 1.585);
   nh.param<double>("Sick2Pose_z", laserSick2Pose.pose.position.z, 1.284);
   nh.param<double>("Sick2Orientation_x", laserSick2Pose.pose.orientation.x, 0.0);
   nh.param<double>("Sick2Orientation_y", laserSick2Pose.pose.orientation.y, 0.0);
   nh.param<double>("Sick2Orientation_z", laserSick2Pose.pose.orientation.z, 0.0);
   nh.param<double>("Sick2Orientation_w", laserSick2Pose.pose.orientation.w, 1.0);
	// Camera Videre
   nh.param<double>("ViderePose_x", cameraViderePose.pose.position.x, 0.0);
   nh.param<double>("ViderePose_y", cameraViderePose.pose.position.y, 1.585);
   nh.param<double>("ViderePose_z", cameraViderePose.pose.position.z, 0.982); //+ raio da camera
   nh.param<double>("VidereOrientation_x", cameraViderePose.pose.orientation.x, 0.0);
   nh.param<double>("VidereOrientation_y", cameraViderePose.pose.orientation.y, 0.0);
   nh.param<double>("VidereOrientation_z", cameraViderePose.pose.orientation.z, 0.0);
   nh.param<double>("VidereOrientation_w", cameraViderePose.pose.orientation.w, 1.0);
	// Antena GPS
   nh.param<double>("AntenaGpsPose_x", gpsAntenaPose.pose.position.x, 0.0);
   nh.param<double>("AntenaGpsPose_y", gpsAntenaPose.pose.position.y, 0.14);
   nh.param<double>("AntenaGpsPose_z", gpsAntenaPose.pose.position.z, 1.24);
   nh.param<double>("AntenaGpsOrientation_x", gpsAntenaPose.pose.orientation.x, 0.0);
   nh.param<double>("AntenaGpsOrientation_y", gpsAntenaPose.pose.orientation.y, 0.0);
   nh.param<double>("AntenaGpsOrientation_z", gpsAntenaPose.pose.orientation.z, 0.0);
   nh.param<double>("AntenaGpsOrientation_w", gpsAntenaPose.pose.orientation.w, 1.0);
	// IMU VG400, ainda precisa ser medido
   nh.param<double>("IMUPose_x", imuVG400Pose.pose.position.x, 1.0);
   nh.param<double>("IMUPose_y", imuVG400Pose.pose.position.y, 1.0);
   nh.param<double>("IMUPose_z", imuVG400Pose.pose.position.z, 1.0);
   nh.param<double>("IMUOrientation_x", imuVG400Pose.pose.orientation.x, 0.0);
   nh.param<double>("IMUOrientation_y", imuVG400Pose.pose.orientation.y, 0.0);
   nh.param<double>("IMUOrientation_z", imuVG400Pose.pose.orientation.z, 0.0);
   nh.param<double>("IMUOrientation_w", imuVG400Pose.pose.orientation.w, 1.0);
	// Xsens MTi-G-700 Unidade Inercial, ainda precisa ser medido
   nh.param<double>("xsensPose_x", xsensPose.pose.position.x, 1.0);
   nh.param<double>("xsensPose_y", xsensPose.pose.position.y, 1.0);
   nh.param<double>("xsensPose_z", xsensPose.pose.position.z, 1.0);
   nh.param<double>("xsensOrientation_x", xsensPose.pose.orientation.x, 0.0);
   nh.param<double>("xsensOrientation_y", xsensPose.pose.orientation.y, 0.0);
   nh.param<double>("xsensOrientation_z", xsensPose.pose.orientation.z, 0.0);
   nh.param<double>("xsensOrientation_w", xsensPose.pose.orientation.w, 1.0);
	// Xsens MTi-G-700 Antena, ainda precisa ser medido
   nh.param<double>("xsensGpsAntenaPose_x", xsensGpsAntenaPose.pose.position.x, 1.0);
   nh.param<double>("xsensGpsAntenaPose_y", xsensGpsAntenaPose.pose.position.y, 1.0);
   nh.param<double>("xsensGpsAntenaPose_z", xsensGpsAntenaPose.pose.position.z, 1.0);
   nh.param<double>("xsensGpsAntenaOrientation_x", xsensGpsAntenaPose.pose.orientation.x, 0.0);
   nh.param<double>("xsensGpsAntenaOrientation_y", xsensGpsAntenaPose.pose.orientation.y, 0.0);
   nh.param<double>("xsensGpsAntenaOrientation_z", xsensGpsAntenaPose.pose.orientation.z, 0.0);
   nh.param<double>("xsensGpsAntenaOrientation_w", xsensGpsAntenaPose.pose.orientation.w, 1.0);
	//Base_footprint, ainda precisa ser medido
   nh.param<double>("HeightBaseLink", footprintPose.pose.position.z, -0.5);
 	footprintPose.pose.orientation.x=footprintPose.pose.orientation.y=footprintPose.pose.orientation.z=0.0;
	footprintPose.pose.orientation.w=1.0;

	return;

} // IniciaConstantPoses

void SendConstantTFs (){
	// if (!firstFix)PublishTF (firstOdom, utmTransformName, mapTransformName);
	// Envia as posições dos sensores fixos.
	PublishTF (laserHokuyoPose, vehicleTransformName, hokuyoTransformName);
	PublishTF (laserSick1Pose, vehicleTransformName, sick1TransformName);
	PublishTF (laserSick2Pose, vehicleTransformName, sick2TransformName);
	PublishTF (cameraViderePose, vehicleTransformName, videreTransformName);
	PublishTF (imuVG400Pose, vehicleTransformName, imuTransformName);
	PublishTF (gpsAntenaPose, vehicleTransformName, antenaGpsTransformName);
	PublishTF (xsensGpsAntenaPose,vehicleTransformName, xsensAntenaGpsTransformName);
	PublishTF (xsensPose, vehicleTransformName, xsensTransformName);
	PublishTF (footprintPose, vehicleTransformName, footprintTransformName);
	return;
} // SendConstantTFs

void SendTFTree (){
	// Envia toda a arvore de TF.
	nav_msgs::Odometry msg;
	
	CopyOdomTo (fusionPose, msg.pose);
	if(pubMapOdom){	SendOdomMapCorrection (msg, false); }
	if(pubOdomVehicle){
		CopyOdomTo (odomPose, msg.pose);
		SendOdom(msg);
	}
	SendConstantTFs ();	
	return;
} // SendTFTree

void CreateTFTree (){
	firstFix = 1;
	// Inicia as mensagens.
	IniciaPoseWithCovariance (odomPose);
	IniciaPoseWithCovariance (gpsPose);
	IniciaPoseWithCovariance (fusionPose);	
	IniciaPoseWithCovariance (firstOdom);
	IniciaConstantPoses ();
	SendTFTree ();
	return;
} // CreateTFTree

int main (int argc, char **argv){
	// Inicia o modulo no ros.
	ros::init(argc, argv, SUPER_TF_NODE_NAME);
	ros::NodeHandle n;
	std::string odom_topic, odomMapCorrection_topic, cardata_topic;
	
	CreateTFTree ();

	// Inicia os nomes dos topicos.
	ros::NodeHandle nh("~");
	nh.param<std::string>("cardata_topic", cardata_topic, "verocarfreedom/car_data");
   	nh.param<std::string>("odomMapCorrection_topic", odomMapCorrection_topic, "/fusion2localise2d/pose2d");

	nh.param<std::string>("odom_topic", odom_topic, "/pose");
  	nh.param<bool>("enableMapToOdom", pubMapOdom , true);
   	nh.param<bool>("enableOdomToVehicle", pubOdomVehicle , true); 
	ROS_DEBUG_STREAM("pubMapOdom: " << pubMapOdom << "\nOdomToVehicle" << pubOdomVehicle );

	// Inicia os nomes das transformacoes.
	IniciaTransformsNames ();

	// Gera e envia os tfs conforme as posições forem alteradas.


	ros::Subscriber odom_sub = n.subscribe(odom_topic, 100, SendOdom);
	ros::Subscriber odomMapCorrection_sub = n.subscribe(odomMapCorrection_topic, 100, SendOdomMapCorrection);
	ros::Subscriber cardata_sub = n.subscribe(cardata_topic, 100, SendFrontWheelTF);

	// Manda os tfs de tempos em tempos para a arvore sempre estar viva.
	ros::Rate cycle(25);
	while (ros::ok()) {
		SendTFTree ();
		cycle.sleep();
		ros::spinOnce ();
	}
	return 0;
}
