#include "ptu_classes.h"

ptu_class* ptu_class::instance = NULL;

ptu_class::ptu_class(Publisher p1, Publisher p2, string port)
{
	vel_pub = new Publisher();
	*vel_pub = p1;
	pose_pub = new Publisher();
	*pose_pub = p2;
	ptu = new CPtuDPerception();
	ptu->init(port);
	ptu->maxPosQ('P', max_posP);
	ptu->maxPosQ('T', max_posT);
	v_angularP = 0;
	v_angularT = 0;
}

ptu_class* ptu_class::uniqueInst(Publisher p1, Publisher p2, string port)
{
	if(instance == 0){
		instance = new ptu_class(p1, p2, port);
	}
	
	return instance;
}

CPtuDPerception* ptu_class::getptu()
{
	return ptu;
}
	
void ptu_class::PTU46_callback(const geometry_msgs::Twist &msg)
{
	geometry_msgs::Twist vel;
	geometry_msgs::Pose pose;
	double vectorX, vectorY, vectorZ;
	double armP = 0.08;
	double armT = 0.04;
	double heightT = 0.06; 
	Rotation rot;
	
	ptu->speedQ('P', vel.angular.z);
	if(max_posP < 0)
		vel.angular.z = - vel.angular.z;
	
	ptu->speedQ('T', vel.angular.y);
	if(max_posT < 0)
		vel.angular.y = - vel.angular.y;
	
	vel_pub->publish(vel);
	
	if(msg.angular.z < 0){
		v_angularP = - msg.angular.z;
		if(max_posP > 0)
			max_posP = - max_posP;
		
		ptu->speed('P', v_angularP);
		ptu->moveToAbsPos('P', max_posP);
	}
	else if(msg.angular.z > 0){
		v_angularP = msg.angular.z;
		if(max_posP < 0)
			max_posP = - max_posP;
		
		ptu->speed('P', v_angularP);
		ptu->moveToAbsPos('P', max_posP);
	}
	else if(msg.angular.z == 0){
		v_angularP = 0;
		ptu->speed('P', v_angularP);
		ptu->halt('P');
	}
	
	ptu->absPosQ('P', pos_atualP);
	ROS_INFO("Pan: pos atual = %.2lf --- vel ang = %.2lf", pos_atualP, msg.angular.z);
	
	if(msg.angular.y < 0){
		v_angularT = - msg.angular.y;
		if(max_posT > 0)
			max_posT = - max_posT;
		
		ptu->speed('T', v_angularT);
		ptu->moveToAbsPos('T', max_posT);
	}
	else if(msg.angular.y > 0){
		v_angularT = msg.angular.y;
		if(max_posT < 0)
			max_posT = - max_posT;
		
		ptu->speed('T', v_angularT);
		ptu->moveToAbsPos('T', max_posT);
	}
	else if(msg.angular.y == 0){
		v_angularT = 0;
		ptu->speed('T', v_angularT);
		ptu->halt('T');
	}
	
	ptu->absPosQ('T', pos_atualT);
	ROS_INFO("Tilt: pos atual = %.2lf --- vel ang = %.2lf", pos_atualT, msg.angular.y);
	
	vectorX = armP * cos(pos_atualP - PI/2) + armT * sin(pos_atualT) * cos(pos_atualP);
	vectorY = armP * sin(pos_atualP - PI/2) + armT * sin(pos_atualT) * sin(pos_atualP);
	vectorZ = heightT + armT * cos(pos_atualT);
	
	transform.setOrigin(Vector3(vectorX, vectorY, vectorZ));
    	transform.setRotation(Quaternion(0, 0, pos_atualP));
    	br.sendTransform(StampedTransform(transform, ros::Time::now(), "base_PanTilt", "base_PanTilt_arm"));
    	
    	transform.setOrigin(Vector3(0, 0, 0));
    	transform.setRotation(Quaternion(pos_atualT, 0, 0));
    	br.sendTransform(StampedTransform(transform, ros::Time::now(), "base_PanTilt_arm", "PanTilt_arm"));
    	
    	pose.position.x = vectorX;
    	pose.position.y = vectorY;
    	pose.position.z = vectorZ;
    	
    	rot = Rotation::RPY(0, pos_atualT, pos_atualP);
    	rot.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    	
    	pose_pub->publish(pose);
}
