#include "ransac_classes.hpp"

laser* laser::instance = 0;

laser::laser(Publisher p, Publisher p2, bool wp, NodeHandle node)
{
	listener = new TransformListener();
	pub_lines = new Publisher();
	pub_bisec = new Publisher();
	*pub_lines = p;
	*pub_bisec = p2;
	watchdog = new WatchDog_ransac(node);
	watchdog->StartTimer(DURATION);
	winAngle = 0;
	last_trajAngle = 0;
	last_winAngle = 0;
	if(wp){
		window = new CDisplayWindowPlots("RANSAC", 800, 600);
		window->axis(-5, 5, -3, 13, true);
	}
	else{
		window = NULL;
	}
	
}

laser* laser::uniqueInst(Publisher p, Publisher p2, bool wp, NodeHandle node)
{
	if(instance == 0){
		instance = new laser(p, p2, wp, node);
	}
	
	return instance;
}

void laser::setthreshold(double x)
{
	threshold = x;
}

void laser::setp_inliers(double x)
{
	p_inliers = x;
}

void laser::setdataWidth(double x)
{
	dataWidth = x;
}

void laser::setwinWidth(double x)
{
	winWidth = x;
}

void laser::setwinLength(double x)
{
	winLength = x;
}

void laser::setVerbose(bool x)
{
	verbose = x;
}

/*****************************************************************************/

ransacControl* ransacControl::instance = 0;

double ransacControl::getangularVel()
{
	return angularVel;
}

void ransacControl::setangularVel(double x)
{
	angularVel = x;
}

void ransacControl::setdt(double x)
{
	dt = x;
}

double ransacControl::getdt()
{
	return dt;
}

void ransacControl::setv_linear(double x)
{
	v_linear = x;
}

void ransacControl::setKPT(double x)
{
	KPT = x;
}

void ransacControl::setKIT(double x)
{
	KIT = x;
}

void ransacControl::setKRT(double x)
{
	KRT = x;
}

void ransacControl::setKVT(double x)
{
	KVT = x;
}

void ransacControl::setlenght(double x)
{
	lenght = x;
}

void ransacControl::setwhich_car(string x)
{
	which_car = x;
}

string ransacControl::getwhich_car()
{
	return which_car;
}

void ransacControl::publica(const ransac_project::CarCommand msg)
{
	pub->publish(msg);
}

void ransacControl::publica(const geometry_msgs::Twist msg)
{
	pub->publish(msg);
}

ransacControl::ransacControl(Publisher p, bool wp, NodeHandle node)
{
	pub = new Publisher();
	*pub = p;
	watchdog = new WatchDog_ransac(node);
	watchdog->StartTimer(DURATION);
	
	if(wp){
		window = new CDisplayWindowPlots("RANSAC", 800, 600);
		window->axis(-5, 5, -3, 13, true);
	}
	else{
		window = NULL;
	}
	
}

ransacControl* ransacControl::uniqueInst(Publisher p, bool wp, NodeHandle node)
{
	if(instance == 0){
		instance = new ransacControl(p, wp, node);
	}
	
	return instance;
}

/*****************************************************************************/

bool WatchDog_ransac::StartTimer (double duration)
{
	if (this->gethasTimer()) this->gettimer().Timer::~Timer();
	
	this->settimer(this->getn()->createTimer(Duration(duration), WatchDogHolder_ransac(this)));
	this->sethasTimer(true);
	
	return true;
}


int WatchDog_ransac::KillApplication()
{
	if(this->getisAlive()){
		
		this->setisAlive(false);
		return 1;
	}
	
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
	exit(-1);
}

