#include "bounding_box.h"

#include "DataPointsFilters/BoundingBox.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindBoundingBox(py::module& p_module)
		{
			using BoundingBoxDataPointsFilter = BoundingBoxDataPointsFilter<ScalarType>;
			py::class_<BoundingBoxDataPointsFilter, std::shared_ptr<BoundingBoxDataPointsFilter>, DataPointsFilter>(p_module, "BoundingBoxDataPointsFilter")
				.def_static("description", &BoundingBoxDataPointsFilter::description)
				.def_static("availableParameters", &BoundingBoxDataPointsFilter::availableParameters)

				.def_readonly("xMin", &BoundingBoxDataPointsFilter::xMin)
				.def_readonly("xMax", &BoundingBoxDataPointsFilter::xMax)
				.def_readonly("yMin", &BoundingBoxDataPointsFilter::yMin)
				.def_readonly("yMax", &BoundingBoxDataPointsFilter::yMax)
				.def_readonly("zMin", &BoundingBoxDataPointsFilter::zMin)
				.def_readonly("zMax", &BoundingBoxDataPointsFilter::zMax)
				.def_readonly("removeInside", &BoundingBoxDataPointsFilter::removeInside)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &BoundingBoxDataPointsFilter::filter, py::arg("input"))
				.def("inPlaceFilter", &BoundingBoxDataPointsFilter::inPlaceFilter, py::arg("cloud"));
		}
	}
}
