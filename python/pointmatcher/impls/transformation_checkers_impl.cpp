#include "transformation_checkers_impl.h"

#include "pointmatcher/TransformationCheckersImpl.h"

namespace python
{
	namespace pointmatcher
	{
		namespace impls
		{
			void pybindTransformationCheckersImpl(py::module& p_module)
			{
				using TransformationCheckersImpl = TransformationCheckersImpl<ScalarType>;
				py::class_<TransformationCheckersImpl, std::shared_ptr<TransformationCheckersImpl>> pyTransformationCheckersImpl(p_module, "TransformationCheckersImpl");

				using CounterTransformationChecker = TransformationCheckersImpl::CounterTransformationChecker;
				py::class_<CounterTransformationChecker, std::shared_ptr<CounterTransformationChecker>, TransformationChecker>(pyTransformationCheckersImpl, "CounterTransformationChecker")
					.def_static("description", &CounterTransformationChecker::description)
					.def_static("availableParameters", &CounterTransformationChecker::availableParameters)

					.def_readonly("maxIterationCount", &CounterTransformationChecker::maxIterationCount)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("init", &CounterTransformationChecker::init, py::arg("parameters"), py::arg("iterate"))
					.def("check", &CounterTransformationChecker::check, py::arg("parameters"), py::arg("iterate"));

				using DifferentialTransformationChecker = TransformationCheckersImpl::DifferentialTransformationChecker;
				py::class_<DifferentialTransformationChecker, std::shared_ptr<DifferentialTransformationChecker>, TransformationChecker>(pyTransformationCheckersImpl, "DifferentialTransformationChecker")
					.def_static("description", &DifferentialTransformationChecker::description)
					.def_static("availableParameters", &DifferentialTransformationChecker::availableParameters)

					.def_readonly("minDiffRotErr", &DifferentialTransformationChecker::minDiffRotErr)
					.def_readonly("minDiffTransErr", &DifferentialTransformationChecker::minDiffTransErr)
					.def_readonly("smoothLength", &DifferentialTransformationChecker::smoothLength)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("init", &DifferentialTransformationChecker::init, py::arg("parameters"), py::arg("iterate"))
					.def("check", &DifferentialTransformationChecker::check, py::arg("parameters"), py::arg("iterate"));

				using BoundTransformationChecker = TransformationCheckersImpl::BoundTransformationChecker;
				py::class_<BoundTransformationChecker, std::shared_ptr<BoundTransformationChecker>, TransformationChecker>(pyTransformationCheckersImpl, "BoundTransformationChecker")
					.def_static("description", &BoundTransformationChecker::description)
					.def_static("availableParameters", &BoundTransformationChecker::availableParameters)

					.def_readonly("minDiffRotErr", &BoundTransformationChecker::maxRotationNorm)
					.def_readonly("minDiffTransErr", &BoundTransformationChecker::maxTranslationNorm)

					.def(py::init<const Parameters&>(), py::arg("params") = Parameters())
					.def("init", &BoundTransformationChecker::init, py::arg("parameters"), py::arg("iterate"))
					.def("check", &BoundTransformationChecker::check, py::arg("parameters"), py::arg("iterate"));
			}
		}
	}
}
