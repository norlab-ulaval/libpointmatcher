#include "matches.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindMatches(py::class_<PM>& p_class)
		{
			py::class_<Matches> pyMatches(p_class, "Matches");

			pyMatches.doc() = R"pbdoc(
Result of the data-association step (Matcher::findClosests), before outlier rejection.

This class holds a list of associated reference identifiers, along with the corresponding \e squared distance, for all points in the reading.
A single point in the reading can have one or multiple matches.
)pbdoc";

			using Dists = Matches::Dists;
			using Ids = Matches::Ids;

			pyMatches
//			.def_readonly_static("InvalidId", &Matches::InvalidId, "In case of too few matches the ids are filled with InvalidId") FIXME
//			.def_readonly_static("InvalidDist", &Matches::InvalidDist, "In case of too few matches the dists are filled with InvalidDist") FIXME

				.def(py::init<>()).def(py::init<const Dists&, const Ids>(), py::arg("dists"), py::arg("ids"))
				.def(py::init<const int, const int>(), py::arg("knn"), py::arg("pointsCount"))

				.def_readwrite("dists", &Matches::dists, "squared distances to closest points")
				.def_readwrite("ids", &Matches::ids, "identifiers of closest points")

				.def("getDistsQuantile", &Matches::getDistsQuantile, py::arg("quantile"))
				.def("getMedianAbsDeviation", &Matches::getMedianAbsDeviation)
				.def("getStandardDeviation", &Matches::getStandardDeviation);
		}
	}
}
