#include "data_points.h"

namespace python
{
	namespace pointmatcher
	{
		void pybindDataPoints(py::class_<PM>& p_class)
		{
			using View = DataPoints::View;
			using TimeView = DataPoints::TimeView;
			using ConstView = DataPoints::ConstView;
			using TimeConstView = DataPoints::TimeConstView;

			py::class_<DataPoints> pyDataPoints(p_class, "DataPoints");

			pyDataPoints.doc() = R"pbdoc(
A point cloud

For every point, it has features and, optionally, descriptors.
Features are typically the coordinates of the point in the space.
Descriptors contain information attached to the point, such as its color, its normal vector, etc.
In both features and descriptors, every point can have multiple channels.
Every channel has a dimension and a name.
For instance, a typical 3D cloud might have the channels \c x, \c y, \c z, \c w of dimension 1 as features (using homogeneous coordinates), and the channel \c normal of size 3 as descriptor.
There are no sub-channels, such as \c normal.x, for the sake of simplicity.
Moreover, the position of the points is in homogeneous coordinates because they need both translation and rotation, while the normals need only rotation.
All channels contain scalar values of type ScalarType.)pbdoc";

			py::class_<Label>(pyDataPoints, "Label").def_readwrite("text", &Label::text, "name of the label")
				.def_readwrite("span", &Label::span, "number of data dimensions the label spans")

				.def(py::init<const std::string&, const size_t>(), py::arg("text") = "", py::arg("span") = 0)
				.def("__eq__", &Label::operator==, py::arg("that"));

			py::bind_vector<Labels>(pyDataPoints, "Labels", "A vector of Label")
				.def(py::init<>(), "Construct empty Labels")
				.def(py::init<const Label&>(), py::arg("label"), "Construct Labels with a single Label in it")

				.def("contains", &Labels::contains, py::arg("text"), "Return whether there is a label named text")
				.def("totalDim", &Labels::totalDim, "Return the sum of the spans of each label")
				.def("__str__", [](const Labels& self)
				{
					std::ostringstream oss;
					oss << self;
					return oss.str();
				});

			using InvalidField = DataPoints::InvalidField;
			py::register_exception<InvalidField>(pyDataPoints, "InvalidField");

			pyDataPoints
				// Constructors from descriptions (reserve memory)
				.def(py::init<>())

				.def(py::init<const Labels&, const Labels&, const size_t>(), py::arg("featureLabels"), py::arg("descriptorLabels"), py::arg("pointCount"))

				.def(py::init<const Labels&, const Labels&, const Labels&, const size_t>(), py::arg("featureLabels"), py::arg("descriptorLabels"), py::arg("timeLabels"), py::arg("pointCount"))

					// Copy constructor
				.def(py::init<const DataPoints&>())

					// Copy constructors from partial data
				.def(py::init<const Matrix&, const Labels&>(), py::arg("features"), py::arg("featureLabels"))

				.def(py::init<const Matrix&, const Labels&, const Matrix&, const Labels&>(), py::arg("features"), py::arg("featureLabels"), py::arg("descriptors"), py::arg("descriptorLabels"))

				.def(py::init<const Matrix&, const Labels&, const Matrix&, const Labels&, const PM::Int64Matrix&, const Labels&>(), py::arg("features"), py::arg("featureLabels"), py::arg("descriptors"), py::arg("descriptorLabels"), py::arg("times"), py::arg("timeLabels"))

				.def("__eq__", &DataPoints::operator==, py::arg("that"))

				.def("getNbPoints", &DataPoints::getNbPoints).def("getEuclideanDim", &DataPoints::getEuclideanDim)
				.def("getHomogeneousDim", &DataPoints::getHomogeneousDim)
				.def("getNbGroupedDescriptors", &DataPoints::getNbGroupedDescriptors)
				.def("getDescriptorDim", &DataPoints::getDescriptorDim).def("getTimeDim", &DataPoints::getTimeDim)

				.def("save", &DataPoints::save, py::arg("fileName"), py::arg("binary") = false)
				.def_static("load", &DataPoints::load, py::arg("filename"))

				.def("concatenate", &DataPoints::concatenate, py::arg("dp"))
				.def("conservativeResize", &DataPoints::conservativeResize, py::arg("pointCount"))
				.def("createSimilarEmpty", (DataPoints (DataPoints::*)() const) &DataPoints::createSimilarEmpty)
				.def("createSimilarEmpty", (DataPoints (DataPoints::*)(DataPoints::Index) const) &DataPoints::createSimilarEmpty, py::arg("pointCount"))
				.def("setColFrom", &DataPoints::setColFrom, py::arg("thisCol"), py::arg("that"), py::arg("thatCol"))
				.def("swapCols", &DataPoints::swapCols, py::arg("iCol"), py::arg("jCol"))

				.def("allocateFeature", &DataPoints::allocateFeature, py::arg("name"), py::arg("dim"))
				.def("allocateFeatures", &DataPoints::allocateFeatures, py::arg("newLabels"))
				.def("addFeature", &DataPoints::addFeature, py::arg("name"), py::arg("newFeature"))
				.def("removeFeature", &DataPoints::removeFeature, py::arg("name"))
				.def("getFeatureCopyByName", &DataPoints::getFeatureCopyByName, py::arg("name"))
				.def("getFeatureViewByName_const", (ConstView (DataPoints::*)(const std::string&) const) &DataPoints::getFeatureViewByName, py::arg("name"))
				.def("getFeatureViewByName", (View (DataPoints::*)(const std::string&)) &DataPoints::getFeatureViewByName, py::arg("name"))
				.def("getFeatureRowViewByName_const", (ConstView (DataPoints::*)(const std::string&, const unsigned) const) &DataPoints::getFeatureRowViewByName, py::arg("name"), py::arg("row"))
				.def("getFeatureRowViewByName", (View (DataPoints::*)(const std::string&, const unsigned)) &DataPoints::getFeatureRowViewByName, py::arg("name"), py::arg("row"))
				.def("featureExists", (bool (DataPoints::*)(const std::string&) const) &DataPoints::featureExists, py::arg("name"))
				.def("featureExists", (bool (DataPoints::*)(const std::string&, const unsigned) const) &DataPoints::featureExists, py::arg("name"), py::arg("dim"))
				.def("getFeatureDimension", &DataPoints::getFeatureDimension, py::arg("name"))
				.def("getFeatureStartingRow", &DataPoints::getFeatureStartingRow, py::arg("name"))

				.def("allocateDescriptor", &DataPoints::allocateDescriptor, py::arg("name"), py::arg("dim"))
				.def("allocateDescriptors", &DataPoints::allocateDescriptors, py::arg("newLabels"))
				.def("addDescriptor", &DataPoints::addDescriptor, py::arg("name"), py::arg("newDescriptor"))
				.def("removeDescriptor", &DataPoints::removeDescriptor, py::arg("name"))
				.def("getDescriptorCopyByName", &DataPoints::getDescriptorCopyByName, py::arg("name"))
				.def("getDescriptorViewByName_const", (ConstView (DataPoints::*)(const std::string&) const) &DataPoints::getDescriptorViewByName, py::arg("name"))
				.def("getDescriptorViewByName", (View (DataPoints::*)(const std::string&)) &DataPoints::getDescriptorViewByName, py::arg("name"))
				.def("getDescriptorRowViewByName_const", (ConstView (DataPoints::*)(const std::string&, const unsigned) const) &DataPoints::getDescriptorRowViewByName, py::arg("name"), py::arg("row"))
				.def("getDescriptorRowViewByName", (View (DataPoints::*)(const std::string&, const unsigned)) &DataPoints::getDescriptorRowViewByName, py::arg("name"), py::arg("row"))
				.def("descriptorExists", (bool (DataPoints::*)(const std::string&) const) &DataPoints::descriptorExists)
				.def("descriptorExists", (bool (DataPoints::*)(const std::string&, const unsigned) const) &DataPoints::descriptorExists)
				.def("getDescriptorDimension", &DataPoints::getDescriptorDimension, py::arg("name"))
				.def("getDescriptorStartingRow", &DataPoints::getDescriptorStartingRow, py::arg("name"))
				.def("assertDescriptorConsistency", &DataPoints::assertDescriptorConsistency)

				.def("allocateTime", &DataPoints::allocateTime, py::arg("name"), py::arg("dim"))
				.def("allocateTimes", &DataPoints::allocateTimes, py::arg("newLabels"))
				.def("addTime", &DataPoints::addTime, py::arg("name"), py::arg("newTime"))
				.def("removeTime", &DataPoints::removeTime, py::arg("name"))
				.def("getTimeCopyByName", &DataPoints::getTimeCopyByName, py::arg("name"))
				.def("getTimeViewByName_const", (TimeConstView (DataPoints::*)(const std::string&) const) &DataPoints::getTimeViewByName, py::arg("name"))
				.def("getTimeViewByName", (TimeView (DataPoints::*)(const std::string&)) &DataPoints::getTimeViewByName, py::arg("name"))
				.def("getTimeRowViewByName_const", (TimeConstView (DataPoints::*)(const std::string&, const unsigned) const) &DataPoints::getTimeRowViewByName, py::arg("name"), py::arg("row"))
				.def("getTimeRowViewByName", (TimeView (DataPoints::*)(const std::string&, const unsigned)) &DataPoints::getTimeRowViewByName, py::arg("name"), py::arg("row"))
				.def("timeExists", (bool (DataPoints::*)(const std::string&) const) &DataPoints::timeExists, py::arg("name"))
				.def("timeExists", (bool (DataPoints::*)(const std::string&, const unsigned) const) &DataPoints::timeExists, py::arg("name"), py::arg("dim"))
				.def("getTimeDimension", &DataPoints::getTimeDimension, py::arg("name"))
				.def("getTimeStartingRow", &DataPoints::getTimeStartingRow, py::arg("name"))
				.def("assertTimesConsistency", &DataPoints::assertTimesConsistency)
//			.def("__str__)", [](const DataPoints& self)
//			{
//				std::ostringstream oss;
//
//				oss << self.features << '\n';
//
//				for(const Label& l : self.featureLabels)
//				{
//					oss << "featureLabels : ";
//					oss << "text : " << l.text;
//					oss << " span : " << l.span << '\n';
//				}
//
//				oss << self.descriptors << '\n';
//
//				for(const Label& l : self.descriptorLabels)
//				{
//					oss << "descriptorLabels : ";
//					oss << "text : " << l.text;
//					oss << " span : " << l.span << '\n';
//				}
//
//				oss << self.times;
//
//				for(const Label& l : self.timeLabels)
//				{
//					oss << "timeLabels : ";
//					oss << "text : " << l.text;
//					oss << " span : " << l.span << '\n';
//				}
//
//				py::print(oss.str());
//			})

				.def_readwrite("features", &DataPoints::features, "features of points in the cloud")
				.def_readwrite("featureLabels", &DataPoints::featureLabels, "labels of features")
				.def_readwrite("descriptors", &DataPoints::descriptors, "descriptors of points in the cloud, might be empty")
				.def_readwrite("descriptorLabels", &DataPoints::descriptorLabels, "labels of descriptors")
				.def_readwrite("times", &DataPoints::times, "time associated to each points, might be empty")
				.def_readwrite("timeLabels", &DataPoints::timeLabels, "labels of times");
		}
	}
}
