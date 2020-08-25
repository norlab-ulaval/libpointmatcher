#include "transformations_impl.h"

#include "pointmatcher/TransformationsImpl.h"

namespace pointmatcher
{
	void pybindTransformationsImpl(py::module& p_module)
	{
		py::class_<TransformationsImpl, std::shared_ptr<TransformationsImpl>> pyTransformationImpl(p_module, "TransformationsImpl");

		py::class_<RigidTransformation, std::shared_ptr<RigidTransformation>, Transformation>
			(pyTransformationImpl, "RigidTransformation")
			.def_static("description", &RigidTransformation::description)

			.def(py::init<>())
			.def("compute", &RigidTransformation::compute, py::arg("input"), py::arg("parameters"))
			.def("checkParameters", &RigidTransformation::checkParameters, py::arg("parameters"))
			.def("correctParameters", &RigidTransformation::correctParameters, py::arg("parameters"));

		py::class_<SimilarityTransformation, std::shared_ptr<SimilarityTransformation>, Transformation>(pyTransformationImpl, "SimilarityTransformation")
			.def_static("description", &SimilarityTransformation::description)

			.def(py::init<>())
			.def("compute", &SimilarityTransformation::compute, py::arg("input"), py::arg("parameters"))
			.def("checkParameters", &SimilarityTransformation::checkParameters, py::arg("parameters"))
			.def("correctParameters", &SimilarityTransformation::correctParameters, py::arg("parameters"));

		py::class_<PureTranslation, std::shared_ptr<PureTranslation>, Transformation>(pyTransformationImpl, "PureTranslation")
			.def_static("description", &PureTranslation::description)

			.def(py::init<>())
			.def("compute", &PureTranslation::compute, py::arg("input"), py::arg("parameters"))
			.def("checkParameters", &PureTranslation::checkParameters, py::arg("parameters"))
			.def("correctParameters", &PureTranslation::correctParameters, py::arg("parameters"));
	}
}