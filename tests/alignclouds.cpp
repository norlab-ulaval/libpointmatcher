// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>

using namespace std;

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Error in command line, usage " << argv[0] << " inputBaseFileName outputBaseFileName" << endl;
		return 1;
	}
	
	typedef MetricSpaceAlignerD MSA;
	MSA::Strategy p;
	
	p.transformations.push_back(new MSA::TransformFeatures());
	//p.transformations.push_back(new MSA::TransformDescriptors());
	
	p.readingDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.02, true, false));
	p.referenceDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.05, true, false));
	p.referenceDataPointsFilters.push_back(new MSA::SurfaceNormalDataPointsFilter(15, 0, true, false, false, false, false));
	
	p.matcher = new MSA::KDTreeMatcher();
	
	//p.featureOutlierFilters.push_back(new MSA::MedianDistOutlierFilter(25));
	p.featureOutlierFilters.push_back(new MSA::TrimmedDistOutlierFilter(0.92));
	p.featureOutlierFilters.push_back(new MSA::MinDistOutlierFilter(0.000001));
	
	p.descriptorOutlierFilter = new MSA::NullDescriptorOutlierFilter();

	//p.errorMinimizer = new MSA::PointToPointErrorMinimizer();
	p.errorMinimizer = new MSA::PointToPlaneErrorMinimizer();
	
	p.transformationCheckers.push_back(new MSA::CounterTransformationChecker(200));
	p.transformationCheckers.push_back(new MSA::ErrorTransformationChecker(0.0001, 0.001, 3));
	
	p.inspector = new MSA::VTKFileInspector("/tmp/vtk/debug");
	//p.inspector = new MSA::Inspector;
	
	p.outlierMixingWeight = 1;
	
	typedef MSA::TransformationParameters TP;
	typedef MSA::DataPoints DP;
	
	const string inputBaseFileName(argv[1]);
	const string outputBaseFileName(argv[2]);
	DP lastCloud, newCloud;
	MSA::TransformFeatures tf;
	TP tp;
	for (unsigned frameCounter = 0; frameCounter < 10; ++frameCounter)
	{
		const string inputFileName((boost::format("%s.%05d.vtk") % inputBaseFileName % frameCounter).str());
		const string outputFileName((boost::format("%s.%05d.vtk") % outputBaseFileName % frameCounter).str());
		
		ifstream ifs(inputFileName.c_str());
		if (!ifs.good())
		{
			cout << "Stopping at frame " << frameCounter << endl;
			break;
		}
	
		

		newCloud = loadVTK<MSA::ScalarType>(ifs);
		if (frameCounter == 0)
		{
			tp = TP::Identity(newCloud.features.rows(), newCloud.features.rows());
		}	
		
		if (frameCounter != 0)
		{
			tp = MSA::icp(tp, newCloud, lastCloud, p);
			newCloud = tf.compute(newCloud, tp);
		}
		
		saveVTK<MSA::ScalarType>(newCloud, outputFileName);
		lastCloud = newCloud;
	}
	return 0;
}
