#include "octree_grid.h"

#include "DataPointsFilters/OctreeGrid.h"

namespace python
{
	namespace datapointsfilters
	{
		void pybindOctreeGrid(py::module& p_module)
		{
			using OctreeGridDataPointsFilter = OctreeGridDataPointsFilter<ScalarType>;
			py::class_<OctreeGridDataPointsFilter, std::shared_ptr<OctreeGridDataPointsFilter>, DataPointsFilter> octreegridClass(p_module, "OctreeGridDataPointsFilter");

			octreegridClass.doc() = R"pbdoc(
Data Filter based on Octree representation

Processings are applyed via a Visitor through Depth-first search in the Octree (DFS)
i.e. for each node, the Visitor/Callback is called
)pbdoc";

			using FirstPtsSampler = OctreeGridDataPointsFilter::FirstPtsSampler;
			py::class_<FirstPtsSampler> firstptssamplerClass(octreegridClass, "FirstPtsSampler", "Visitors class to apply processing");

			firstptssamplerClass.def_readwrite("idx", &FirstPtsSampler::idx)
//			.def_readwrite("pts", &FirstPtsSampler::pts, py::return_value_policy::reference) FIXME
				.def_readwrite("mapidx", &FirstPtsSampler::mapidx, "Build map of (old index to new index), in case we sample pts at the begining of the pointcloud")

				.def(py::init<DataPoints&>(), py::arg("dp"))

//			.def("__call__", &FirstPtsSampler::operator()<2>, py::arg("oc")) FIXME
//			.def("__call__", &FirstPtsSampler::operator()<3>, py::arg("oc")) FIXME
				.def("finalize", &FirstPtsSampler::finalize);

			using RandomPtsSampler = OctreeGridDataPointsFilter::RandomPtsSampler;
			py::class_<RandomPtsSampler, FirstPtsSampler>(firstptssamplerClass, "RandomPtsSampler")
				.def_readonly("seed", &RandomPtsSampler::seed)

				.def(py::init<DataPoints&>(), py::arg("dp"))
				.def(py::init<DataPoints&, const std::size_t>(), py::arg("dp"), py::arg("seed_"))

//			.def("__call__", &RandomPtsSampler::operator()<2>, py::arg("oc")) FIXME
//			.def("__call__", &RandomPtsSampler::operator()<3>, py::arg("oc")) FIXME
				.def("finalize", &RandomPtsSampler::finalize);

			using CentroidSampler = OctreeGridDataPointsFilter::CentroidSampler;
			py::class_<CentroidSampler, FirstPtsSampler>(firstptssamplerClass, "CentroidSampler")
				.def(py::init<DataPoints&>(), py::arg("dp"));

//			.def("__call__", &CentroidSampler::operator()<2>, py::arg("oc")) FIXME
//			.def("__call__", &CentroidSampler::operator()<3>, py::arg("oc")); FIXME

			using MedoidSampler = OctreeGridDataPointsFilter::MedoidSampler;
			py::class_<MedoidSampler, FirstPtsSampler>(firstptssamplerClass, "MedoidSampler", "Nearest point from the centroid (contained in the cloud)")
				.def(py::init<DataPoints&>(), py::arg("dp"));

//			.def("__call__", &RandomPtsSampler::operator()<2>, py::arg("oc")) FIXME
//			.def("__call__", &RandomPtsSampler::operator()<3>, py::arg("oc")); FIXME

			using SamplingMethod = OctreeGridDataPointsFilter::SamplingMethod;
			py::enum_<SamplingMethod>(octreegridClass, "SamplingMethod").value("FIRST_PTS", SamplingMethod::FIRST_PTS)
				.value("RAND_PTS", SamplingMethod::RAND_PTS).value("CENTROID", SamplingMethod::CENTROID)
				.value("MEDOID", SamplingMethod::MEDOID).export_values();

			octreegridClass.def_static("description", &OctreeGridDataPointsFilter::description)
				.def_static("availableParameters", &OctreeGridDataPointsFilter::availableParameters)

				.def_readwrite("centerX", &OctreeGridDataPointsFilter::buildParallel)
				.def_readwrite("centerY", &OctreeGridDataPointsFilter::maxPointByNode)
				.def_readwrite("centerY", &OctreeGridDataPointsFilter::maxSizeByNode)
				.def_readonly("centerZ", &OctreeGridDataPointsFilter::samplingMethod)

				.def(py::init<const Parameters&>(), py::arg("params") = Parameters(), "Constructor, uses parameter interface")

				.def("filter", &OctreeGridDataPointsFilter::filter)
				.def("inPlaceFilter", &OctreeGridDataPointsFilter::inPlaceFilter);
		}
	}
}
