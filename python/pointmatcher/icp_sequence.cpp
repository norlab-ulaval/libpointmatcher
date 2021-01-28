#include <fstream>

#include "icp_sequence.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindICPSequence(py::class_<PM>& p_class)
		{
			py::class_<ICPSequence, ICP> pyICPSequence(p_class, "ICPSequence");

			pyICPSequence.doc() = R"pbdoc(
ICP alogrithm, taking a sequence of clouds and using a map
Warning: used with caution, you need to set the map manually.
)pbdoc";

			pyICPSequence.def(py::init<>())
				.def("__call__", (TransformationParameters(ICPSequence::*)(const DataPoints&)) &ICPSequence::operator(), py::arg("cloudIn"))
				.def("__call__", (TransformationParameters(ICPSequence::*)(const DataPoints&, const TransformationParameters&)) &ICPSequence::operator(), py::arg("cloudIn"), py::arg("initialTransformationParameters"))
				.def("compute", &ICPSequence::compute, py::arg("cloudIn"), py::arg("initialTransformationParameters"))

				.def("hasMap", &ICPSequence::hasMap).def("setMap", &ICPSequence::setMap)
				.def("clearMap", &ICPSequence::clearMap).def("setDefault", &ICPSequence::setDefault)
				.def("loadFromYaml", [](ICPSequence& self, const std::string& in)
				{
					std::ifstream ifs(in.c_str());
					self.loadFromYaml(ifs);
				}).def("getPrefilteredInternalMap", &ICPSequence::getPrefilteredInternalMap)
				.def("getPrefilteredMap", &ICPSequence::getPrefilteredMap);
		}
	}
}
