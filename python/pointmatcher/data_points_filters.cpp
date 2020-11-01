#include <fstream>

#include "data_points_filters.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindDataPointsFilters(py::class_<PM>& p_class)
		{
			py::bind_vector<DataPointsFilters>(p_class, "DataPointsFilters")
				.def(py::init<>(), "Construct an empty chain").def(py::init([](const std::string& in)
				{
					std::ifstream ifs(in.c_str());
					return std::unique_ptr<DataPointsFilters>(new DataPointsFilters(ifs));
				}), "Construct a chain from a YAML file")

				.def("init", &DataPointsFilters::init, "Init the chain")
				.def("apply", &DataPointsFilters::apply, py::arg("cloud"), "Apply this chain to cloud, mutates cloud");
		}
	}
}
