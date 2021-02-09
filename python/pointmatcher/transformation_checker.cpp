#include "transformation_checker.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindTransformationChecker(py::class_<PM>& p_class)
		{
			py::class_<TransformationChecker, std::shared_ptr<TransformationChecker>, Parametrizable> pyTransformationChecker(p_class, "TransformationChecker");
			pyTransformationChecker.doc() = R"pbdoc(
A transformation checker can stop the iteration depending on some conditions.

For example, a condition can be the number of times the loop was executed, or it can be related to the matching error.
		Because the modules can be chained, we defined that the relation between modules must agree through an OR-condition, while all AND-conditions are defined within a single module.
)pbdoc";

			pyTransformationChecker.def("getLimits", &TransformationChecker::getLimits)
				.def("getConditionVariables", &TransformationChecker::getConditionVariables)
				.def("getLimitNames", &TransformationChecker::getLimitNames)
				.def("getConditionVariableNames", &TransformationChecker::getConditionVariableNames);
		}
	}
}
