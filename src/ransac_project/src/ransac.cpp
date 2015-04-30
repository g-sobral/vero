#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ransac_project/BorderLines.h"
#include "ransac_project/Bisectrix.h"
#include "ransac_classes.hpp"
#include "ransac_2Dline.hpp"
#include "ransac_lib.hpp"

#include <iostream>
#include <fstream>
	
ofstream outFile;

void laser::laserCallback(const sensor_msgs::LaserScan& msg)
{
	watchdog->IsAlive();

	vector<float> x_left, x_right, y_left, y_right;
	vector<float> x_l, x_r, y_l, y_r, lineR(3), lineL(3);
	float modelL[3];
	float modelR[3];
	float **dR, **dL;
	float theta;
	int ret, inliersR, inliersL, i;

	tf::TransformListener listener(ros::Duration(10));
	geometry_msgs::PointStamped laser_point;
	laser_point.header.stamp = Time();
	laser_point.header.frame_id = msg.header.frame_id;
	//laser_point.point.z = 1; /*valor arbitrario para teste sem o carro*/

	for(unsigned int i = 0; i < msg.ranges.size(); i++){
		if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
			theta = msg.angle_min + i * msg.angle_increment; // + PI/2;

			/*polar -> cartesian*/
			laser_point.point.x = msg.ranges[i] * cos(theta);
			laser_point.point.y = msg.ranges[i] * sin(theta);

			winAngle = 0.3*last_winAngle + 0.7*last_trajAngle;
			/*car angle -> trajectory angle*/
			if(winAngle){
				laser_point.point.x = cos(winAngle)*laser_point.point.x +sin(winAngle)*laser_point.point.y;
				laser_point.point.y = -sin(winAngle)*laser_point.point.x +cos(winAngle)*laser_point.point.y;
			}

			try{
				geometry_msgs::PointStamped vero_point;
				listener.waitForTransform("base_link", "hokuyo", ros::Time(), ros::Duration(1.0));
    			listener.transformPoint("base_link", laser_point, vero_point);
    			
    			if(laser_point.point.x < (winLength * dataWidth) && 
					abs(laser_point.point.y) < (winWidth * dataWidth)){
					if(laser_point.point.y > 0){
						if(winAngle){
							x_left.push_back(cos(winAngle)*vero_point.point.x - sin(winAngle)*vero_point.point.y);
							y_left.push_back(sin(winAngle)*vero_point.point.x + cos(winAngle)*vero_point.point.y);
						}
						else{
							x_left.push_back(vero_point.point.x);
							y_left.push_back(vero_point.point.y);
						}
					}
					else if(laser_point.point.y < 0){
						if(winAngle){
							x_right.push_back(cos(winAngle)*vero_point.point.x - sin(winAngle)*vero_point.point.y);
							y_right.push_back(sin(winAngle)*vero_point.point.x + cos(winAngle)*vero_point.point.y);
						}
						else{
							x_right.push_back(vero_point.point.x);
							y_right.push_back(vero_point.point.y);
						}
					}
				}

    			/*ROS_INFO("hokuyo: (%.2f, %.2f. %.2f) -----> vero: (%.2f, %.2f, %.2f) at time %.2f", laser_point.point.x, laser_point.point.y, laser_point.point.z,vero_point.point.x, vero_point.point.y, vero_point.point.z, vero_point.header.stamp.toSec());*/
			}
			catch(TransformException& ex){
				ROS_ERROR("Received an exception trying to transform a point from \"hokuyo\" to \"base_link\": %s", ex.what());
			}
		}	
	}
	
	for(int j = 0; j < x_left.size(); j ++){
		x_l.push_back(x_left[j]);
		y_l.push_back(y_left[j]);
	}

	for(int j = 0; j < x_right.size(); j ++){
		x_r.push_back(x_right[j]);
		y_r.push_back(y_right[j]);
	}

	dR = (float **) malloc(x_right.size() * sizeof(float *));
	if(dR == NULL) { perror("out of memory\n"); exit(0); }
	for(i = 0; i < x_right.size(); i++)
	{
		dR[i] = (float *) malloc(2 * sizeof(float));
		if(dR[i] == NULL) { perror("out of memory\n"); exit(0); }
		dR[i][0] = x_right[i];
		dR[i][1] = y_right[i];
	}
	
	dL = (float **) malloc(x_left.size() * sizeof(float *));
	if(dR == NULL) { perror("out of memory\n"); exit(0); }
	for(i = 0; i < x_left.size(); i++)
	{
		dL[i] = (float *) malloc(2 * sizeof(float));
		if(dL[i] == NULL) { perror("out of memory\n"); exit(0); }
		dL[i][0] = x_left[i];
		dL[i][1] = y_left[i];
	}

	ret = ransac_2Dline(dR, x_right.size(), (x_right.size()/2)-1, threshold, modelR, &inliersR, verbose);
	ret += ransac_2Dline(dL, x_left.size(), (x_left.size()/2)-1, threshold, modelL, &inliersL, verbose);
	
	for(i = 0; i < x_right.size(); i++)
		free(dR[i]);
	free(dR);

	for(i = 0; i < x_left.size(); i++)
		free(dL[i]);
	free(dL);
	
	if(ret == 0){
	
		lineR[0] = modelR[0];
		lineR[1] = modelR[1];
		lineR[2] = modelR[2];
		
		lineL[0] = modelL[0];
		lineL[1] = modelL[1];
		lineL[2] = modelL[2];
		
		vector<double> biModel = bisectrixLine(lineL, lineR);
		vector<float> bi2msg;
		ransac_project::Bisectrix biMsg;

		for(int j = 0; j < biModel.size(); j ++){
			bi2msg.push_back(biModel[j]);
		}
	
		biMsg.bisectrix = bi2msg;
		pub_bisec->publish(biMsg);

		last_winAngle = winAngle;
		last_trajAngle = atan(-biModel[0]/biModel[1]); // inclinacao da bissetriz estimada, em radianos
		outFile << atan(-biModel[0]/biModel[1]) << "\n";

		//x_l, y_l, x_r, y_r sao pontos apenas para a visualização utilizados pelo node dataplot
		ransac_project::BorderLines msg_pub;
		msg_pub.line_left = lineL;
		msg_pub.line_right = lineR;
		msg_pub.x_left = x_l;
		msg_pub.y_left = y_l;
		msg_pub.x_right = x_r;
		msg_pub.y_right = y_r;
		pub_lines->publish(msg_pub);
	}
}

int main(int argc,char **argv){

	ros::init(argc, argv, "ransac");
	ros::NodeHandle ransac_node;
	NodeHandle node("~");

	bool wp = false, verbose = false;
	double threshold, dataWidth, winWidth, winLength;
	
	/*parametros fundamentais para a execução do software*/
	if(!node.getParam("threshold", threshold) || !node.getParam("dataWidth", dataWidth) || !node.getParam("winWidth", winWidth) || !node.getParam("winLength", winLength)){
		
		ROS_ERROR("parameters not specified");
		exit(0);
	}
	
	ros::Publisher ransac_pub = ransac_node.advertise<ransac_project::BorderLines>("ransac_lines", 1);
	
	ros::Publisher bisec_pub = ransac_node.advertise<ransac_project::Bisectrix>("bisec_data", 1); 

	laser* ls = laser::uniqueInst(ransac_pub, bisec_pub, wp, ransac_node);

	ls->setthreshold(threshold);
	ls->setdataWidth(dataWidth);
	ls->setwinWidth(winWidth);
	ls->setwinLength(winLength);
	
	if(node.getParam("verbose", verbose))
		ls->setVerbose(verbose);

	// "log_orca" - orca acquired data
	// "scan" - ros aquired dara
	ros::Subscriber laser_sub = ransac_node.subscribe("scan", 1, &laser::laserCallback, ls);
	// hokuyo: topico "scan"
	// dataset cafe:  topico "log_orca" 

	outFile.open("/home/sobral/angle.txt");

	ros::spin();

	outFile.close();
	
	return 0;
}
