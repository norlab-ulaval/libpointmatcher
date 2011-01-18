// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "Error in command line, usage " << argv[0] << " reference.csv reading.csv" << endl;
		return 1;
	}
	
	typedef MetricSpaceAlignerD MSA;
	MSA::Strategy p;
	
	p.transformations.push_back(new MSA::TransformFeatures());
	p.transformations.push_back(new MSA::TransformDescriptors());
	
	p.readingDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.5, true, false));
	
	p.referenceDataPointsFilters.push_back(new MSA::RandomSamplingDataPointsFilter(0.5, true, false));
	//p.referenceDataPointsFilters.push_back(new MSA::SurfaceNormalDataPointsFilter(10, 0, true, true, true, true, true));
	
	p.matcher = new MSA::KDTreeMatcher();
	
	p.featureOutlierFilters.push_back(new MSA::MaxDistOutlierFilter(0.05));
	//p.featureOutlierFilters.push_back(new MSA::MedianDistOutlierFilter(3));
	//p.featureOutlierFilters.push_back(new MSA::TrimmedDistOutlierFilter(0.85));
	
	p.descriptorOutlierFilter = new MSA::NullDescriptorOutlierFilter();

	p.errorMinimizer = new MSA::PointToPointErrorMinimizer();
	//p.errorMinimizer = new MSA::PointToPlaneErrorMinimizer();
	
	p.transformationCheckers.push_back(new MSA::CounterTransformationChecker(60));
	p.transformationCheckers.push_back(new MSA::ErrorTransformationChecker(0.001, 0.001, 3));
	
	p.inspector = new MSA::VTKFileInspector("test");
	//p.inspector = new MSA::Inspector;
	
	p.outlierMixingWeight = 1;
	
	typedef MSA::TransformationParameters TP;
	typedef MSA::DataPoints DP;
	
	const DP ref(loadCSV<MSA::ScalarType>(argv[1]));
	DP data(loadCSV<MSA::ScalarType>(argv[2]));
	TP t(TP::Identity(data.features.rows(), data.features.rows()));
	
	for (int i = 0; i < data.features.cols(); ++i)
	{
		data.features.block(0, i, 2, 1) = Eigen::Rotation2D<double>(0.2) * data.features.block(0, i, 2, 1);
		data.features(0, i) += 0.2;
		data.features(1, i) -= 0.1;
	}
	
	TP res = MSA::icp(t, data, ref, p);

	return 0;
}
