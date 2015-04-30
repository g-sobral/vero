#include "watchdog/watchdog.hpp"

WatchDog::WatchDog (ros::NodeHandle& nh){
	hasTimer = false;
	isAlive = true;
	n = &nh;
}

bool WatchDog::StartTimer (double duration) {
	if (hasTimer) timer.Timer::~Timer();
	timer = n->createTimer(ros::Duration(duration), WatchDogHolder (this));
	hasTimer = true;
	return true;
}

bool WatchDog::ResetTimer (double duration){
	return StartTimer (duration);
}

bool WatchDog::StopTimer () {
	if (hasTimer) timer.Timer::~Timer();
	hasTimer = false;
	return true;
}

int WatchDog::KillApplication () {
	if (isAlive) {
		isAlive = false;
		return 1;
	}
	exit(-1);
}
