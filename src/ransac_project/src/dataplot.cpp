#include "ransac_lib.hpp"

CDisplayWindowPlots* window;

void linesCallback(const ransac_project::BorderLines& msg)
{
	vector<float> lineR(4), lineL(4);

	plotLine(*window,msg.line_left,"r-2","ransac: left side");
	plotLine(*window,msg.line_right,"b-2","ransac: right side");
	plotPoints(*window,msg.x_left,msg.y_left,"g.2","scan: left side");
	plotPoints(*window,msg.x_right,msg.y_right,"m.2","scan: right side");
}

void bisecCallback(const ransac_project::Bisectrix& msg)
{
	plotLine(*window,msg.bisectrix,"g-2","bisectrix");
}

int main(int argc, char** argv)
{
	init(argc, argv, "dataplot");
	NodeHandle node;
	
	window = new CDisplayWindowPlots("RANSAC", 800, 600);
	window->axis(-5, 15, -5, 5, true);
	
	Subscriber sub1 = node.subscribe("ransac_lines", 1, linesCallback);
	Subscriber sub2 = node.subscribe("bisec_data", 1, bisecCallback);
	
	spin();
	
	return 0;
}
