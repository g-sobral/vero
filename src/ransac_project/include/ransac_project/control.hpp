#include <vector>
#include <math.h>
#include "ros/ros.h"
#include <time.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt::gui;

#define PI 3.14159265

/* LineTracking - Função de controle para seguir uma trajetória dada por uma reta
 *  Entradas: 
 *
 *	line 	: vetor contendo os pontos que definem a trajetória (x1, x2, y1, y2)
 *  v_linear: velocidade linear do veículo
 *  v_angular: velocidade angular do veiculo
 *	dt: intervalo de tempo desde a ultima chamada da funcao
 *  KPT, KIT, KRT, KVT: ganhos do controlador
 *
 *  Saída:
 *
 *  velocidade angular em rad/s

 */

class Controle
{
	private:
		static double errori_[1];
	public:
		static double LineTracking( vector<double> line, const double v_linear, const double v_angular, double dt, double KPT, double KIT, double KRT, double KVT);
};

