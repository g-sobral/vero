#include <stdio.h>
#include "ros/ros.h"

class WatchDog {
 public:
	WatchDog (ros::NodeHandle& nh);
	~WatchDog (){if (hasTimer) timer.Timer::~Timer();};
	bool StartTimer (double duration);
	bool ResetTimer (double duration);
	bool StopTimer ();
	void IsAlive () { isAlive=true; };
	int KillApplication ();
	bool getisAlive(){ return isAlive; };
	void setisAlive(bool x){ isAlive = x; };
	bool gethasTimer(){ return hasTimer; };
	void sethasTimer(bool x){ hasTimer = x; };
	ros::Timer gettimer(){ return timer; };
	void settimer(ros::Timer x){ timer = x; };
	ros::NodeHandle* getn(){ return n; };
	void setn(ros::NodeHandle* x){ n = x; };
 private:
	ros::NodeHandle* n;
	bool hasTimer;
	bool isAlive;
	ros::Timer timer;
}; // WatchDog

class WatchDogHolder {
 public:
	WatchDogHolder (WatchDog* watchdog) {dog = watchdog;};
	void operator ()(const ros::TimerEvent& event)
	{
		dog->KillApplication();
	}
	
	WatchDog* dog;
};
