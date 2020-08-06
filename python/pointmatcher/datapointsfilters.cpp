#include <fstream>

#include "datapointsfilters.h"

namespace pointmatcher
{
	void pybindDataPointsFilters(py::class_<PM>& p_class)
	{
		py::bind_vector<std::vector<std::shared_ptr<DataPointsFilter>>>(p_class, "DataPointsFiltersVector");

		py::class_<DataPointsFilters, std::vector<std::shared_ptr<DataPointsFilter>>>(p_class, "DataPointsFilters",
				"A chain of DataPointsFilter")
			.def(py::init<>())
			.def(py::init([](const std::string& in)
				{
					std::ifstream ifs(in.c_str());
					return std::unique_ptr<DataPointsFilters>(new DataPointsFilters(ifs));
				}))
			.def("init", &DataPointsFilters::init)
			.def("apply", &DataPointsFilters::apply, py::arg("cloud"));
	}
}