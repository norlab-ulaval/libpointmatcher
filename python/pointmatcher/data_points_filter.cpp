#include "data_points_filter.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindDataPointsFilter(py::class_<PM>& p_class)
		{
			py::class_<DataPointsFilter, std::shared_ptr<DataPointsFilter>, Parametrizable>(p_class, "DataPointsFilter", "A data filter takes a point cloud as input, transforms it, and produces another point cloud as output.")
				.def("init", &DataPointsFilter::init)
				.def("filter", &DataPointsFilter::filter, py::arg("input"), "Apply filters to input point cloud.  This is the non-destructive version and returns a copy.")
				.def("inPLaceFilter", &DataPointsFilter::inPlaceFilter, py::arg("cloud"), "Apply these filters to a point cloud without copying.");
		}
	}
}

