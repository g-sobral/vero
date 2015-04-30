#include "ransac_lib.hpp"


vector<double> bisectrixLine(vector<float> l1, vector<float> l2)
{
	vector<double> bisectrix(3), V1(2), V2(2), VY(2);
	double a1, b1, c1, a2, b2, c2;
	double a1k, b1k, c1k, a2k, b2k, c2k, k1, k2;
	double bi1_a, bi1_b, bi1_c, bi2_a, bi2_b, bi2_c, theta1, theta2, modV1, modV2, modVY;
		
	a1 = l1[0];
	b1 = l1[1];
	c1 = l1[2];
	a2 = l2[0];
	b2 = l2[1];
	c2 = l2[2];
	
	k1 = sqrt(a1*a1 + b1*b1);
	k2 = sqrt(a2*a2 + b2*b2);
	
	a1k = a1/k1;
	b1k = b1/k1;
	c1k = c1/k1;
	a2k = a2/k2;
	b2k = b2/k2;
	c2k = c2/k2;
	
	bi1_a = (a1k - a2k);
	bi1_b = (b1k - b2k);
	bi1_c = (c1k - c2k);
	bi2_a = (a1k + a2k);
	bi2_b = (b1k + b2k);
	bi2_c = (c1k + c2k);

	// seleciona a bissetriz com menor angulo em relacao ao eixo Y
	// evitando divisoes por zero
	
	if(bi1_b != 0){
		V1[0] = 10;
		V1[1] = ((-bi1_c - V1[0]*bi1_a)/bi1_b) - (-bi1_c/bi1_b);
	}
	else if(bi1_a != 0){
		V1[1] = 10;
		V1[0] = ((-bi1_c - V1[1]*bi1_b)/bi1_a) - (-bi1_c/bi1_a);
	}
	
	if(bi2_b != 0){
		V2[0] = 10;
		V2[1] = ((-bi2_c - V2[0]*bi2_a)/bi2_b) - (-bi2_c/bi2_b);
	}
	else if(bi2_a != 0){
		V2[1] = 10;
		V2[0] = ((-bi2_c - V2[1]*bi2_b)/bi2_a) - (-bi2_c/bi2_a);
	}
	
	VY[0] = 0;
	VY[1] = 10;
	
	modV1 = sqrt(V1[0]*V1[0] + V1[1]*V1[1]);
	modV2 = sqrt(V2[0]*V2[0] + V2[1]*V2[1]);
	modVY = sqrt(VY[0]*VY[0] + VY[1]*VY[1]);
	
	theta1 = acos(abs(V1[0]*VY[0] + V1[1]*VY[1])/abs(modV1*modVY));
	theta2 = acos(abs(V2[0]*VY[0] + V2[1]*VY[1])/abs(modV2*modVY));

	if(abs(theta1)>=abs(theta2)){
		bisectrix[0] = bi1_a;
		bisectrix[1] = bi1_b;
		bisectrix[2] = bi1_c;
	}
	else{
		bisectrix[0] = bi2_a;
		bisectrix[1] = bi2_b;
		bisectrix[2] = bi2_c;
	}
  
	return bisectrix;
}

void plotLine(mrpt::gui::CDisplayWindowPlots &win, vector<float> line, string format, string name)
{
/*
	vector<double> lx(2), ly(2);
	lx[0]=line[0];
	lx[1]=line[1];
	ly[0]=line[2];
	ly[1]=line[3];
*/
	vector<double> lx(21), ly(21);
	for(int i=0; i<21; i++)
	{
		lx[i] = i-10;
		ly[i] = (-lx[i]*line[0] - line[2])/line[1];
	}
	
	win.plot(lx,ly,format,name);
}

void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector<float> x,vector<float> y, string format, string name)
{
	
	vector<double> lx(x.size()), ly(y.size());
	
	for (unsigned int i=0; i<x.size();i++) {
		lx[i]=x[i];
		ly[i]=y[i];
	}
	
	win.plot(lx,ly,format,name);
}
/*
void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector<float> x,vector<float> y, string format, string name)
{
	win.plot(x,y,format,name);
}
*/


