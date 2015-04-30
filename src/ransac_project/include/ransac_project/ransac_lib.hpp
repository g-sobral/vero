#include <math.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <tr1/functional>

#include <mrpt/gui.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "ransac_project/BorderLines.h"
#include "ransac_project/CarCommand.h"
#include "ransac_project/Bisectrix.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

using namespace ros;
using namespace std;
using namespace std::tr1;
using namespace tf;
using namespace mrpt::gui;

#define PI 3.14159265
#define DURATION 0.5

/* bisectrixLine - calcula a linha bissetriz entre duas linhas, cada uma delas
					definida por dois pontos
	
	Entradas
	l1 	: vetor contendo os coeficientes da primeira linha (a1, b1, c1)
	l2	: vetor contendo os coeficientes da segunda linha (a2, b2, c2)
	
	Saidas
	bisectrix	: vetor contendo os coeficientes que definem a linha bissetriz (ab, bb, cb)
		
*/
vector<double> bisectrixLine(vector<float> l1, vector<float> l2);

/* plotLine - plota uma linha
	
	Entradas
	win		: ponteiro para a janela onde a linha deve ser plotada
	line 	: vetor contendo os pontos da linha a ser plotada (x1, x2, y1, y2)
	format	: string com o formato da linha a ser plotada
	name	: string com o nome da linha a ser plotada
		
*/
void plotLine(mrpt::gui::CDisplayWindowPlots &win, vector<float> line, string format, string name);

void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector<float> x,vector<float> y, string format, string name);

/*
void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector<float> x,vector<float> y, string format, string name);
*/
