#include "ransac_lib.hpp"
#include "watchdog/watchdog.hpp"

class WatchDog_ransac : public WatchDog
{
	public:
		WatchDog_ransac(NodeHandle node) : WatchDog(node){};
		int KillApplication();
		bool StartTimer(double duration);
};

class WatchDogHolder_ransac
{
	public:
		WatchDogHolder_ransac(WatchDog_ransac* watchdog) {dog = watchdog;};
		void operator ()(const TimerEvent& event)
		{
			dog->KillApplication();
		}
	
	WatchDog_ransac* dog;
};

class laser
{
	private:
		static laser* instance;
		TransformListener* listener;
		Publisher* pub_lines;
		Publisher* pub_bisec;
		CDisplayWindowPlots* window;
		WatchDog* watchdog;
		float threshold, p_inliers, dataWidth, winWidth, winLength;
		bool verbose;
		float winAngle;			// angulo utilizado para a divisao da janela
													// direita/esquerda na ultima iteracao
		float last_trajAngle;	// angulo da ultima trajetoria estimada
		float last_winAngle;	
	protected:
		laser(Publisher p, Publisher p2, bool wp, NodeHandle node);
	public:
		static laser* uniqueInst(Publisher p, Publisher p2, bool wp, NodeHandle node);
		void laserCallback(const sensor_msgs::LaserScan& msg);
		void setthreshold(double x);
		void setp_inliers(double x);
		void setdataWidth(double x);
		void setwinWidth(double x);
		void setwinLength(double x);
		void setVerbose(bool x);
};

class ransacControl
{
	private:
		static ransacControl* instance;
		Publisher* pub;
		CDisplayWindowPlots *window;
		WatchDog_ransac* watchdog;
		double angularVel, dt, v_linear, lenght, KPT, KIT, KRT, KVT;
		string which_car;
	protected:
		ransacControl(Publisher p, bool wp, NodeHandle node);
	public:
		ransacControl();
		static ransacControl* uniqueInst(Publisher p, bool wp, NodeHandle node);
		void ransacCallback(const ransac_project::Bisectrix &biMsg);
		void odometryCallback(const nav_msgs::Odometry &Odom_msg);
		void publica(const ransac_project::CarCommand msg);
		void publica(const geometry_msgs::Twist msg);
		void setangularVel(double x);
		double getangularVel();
		void setdt(double x);
		double getdt();
		void setv_linear(double x);
		void setlenght(double x);
		void setKPT(double x);
		void setKIT(double x);
		void setKRT(double x);
		void setKVT(double x);
		void setwhich_car(string x);
		string getwhich_car();
};
