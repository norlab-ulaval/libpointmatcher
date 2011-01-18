// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <boost/progress.hpp>

using namespace std;
typedef MetricSpaceAligner<float> MSA;
typedef MSA::DataPoints DataPoints;

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Usage " << argv[0] << " INPUT OUTPUT\n";
		return 1;
	}
	
	DataPoints d = loadVTK<float>(argv[1]);
	MSA::SamplingSurfaceNormalDataPointsFilter filter(100);
	bool iterate;
	DataPoints d2 = filter.preFilter(d, iterate);
	//MSA::VTKFileInspector inspector(argv[2]); inspector.dumpDataPoints(d2, "cloud");
	saveVTK<float>(d2, argv[2]);
	
	return 0;
}
