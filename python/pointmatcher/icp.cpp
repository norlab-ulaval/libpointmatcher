#include "icp.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindICP(py::class_<PM>& p_class)
		{
			py::class_<ICP, ICPChaineBase>(p_class, "ICP", "ICP algorithm").def(py::init<>())
				.def("__call__", (TransformationParameters (ICP::*)(const DataPoints&, const DataPoints&)) &ICP::operator(), py::arg("readingIn"), py::arg("referenceIn"))
				.def("__call__", (TransformationParameters (ICP::*)(const DataPoints&, const DataPoints&, const TransformationParameters&)) &ICP::operator(), py::arg("readingIn"), py::arg("referenceIn"), py::arg("initialTransformationParameters"))
				.def("compute", &ICP::compute, py::arg("readingIn"), py::arg("referenceIn"), py::arg("initialTransformationParameters"))
				.def("getReadingFiltered", &ICP::getReadingFiltered, "Return the filtered point cloud reading used in the ICP chain");
		}
	}
}
