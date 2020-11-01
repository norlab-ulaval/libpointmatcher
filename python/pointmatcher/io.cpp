#include "io.h"

#include "pointmatcher/IO.h"

PYBIND11_MAKE_OPAQUE(std::vector<PointMatcherIO<ScalarType>::SupportedLabel>) // SupportedLabels
PYBIND11_MAKE_OPAQUE(std::vector<PointMatcherIO<ScalarType>::FileInfo>) // FileInfoVector
PYBIND11_MAKE_OPAQUE(std::vector<PointMatcherIO<ScalarType>::PLYProperty>) // PLYProperties

namespace python
{
	namespace pointmatcher
	{
		void pybindIO(py::module& p_module)
		{
			using PMIO = PointMatcherIO<ScalarType>;
			py::class_<PMIO> pyPointMatcherIO(p_module, "PointMatcherIO", "IO Functions and classes that are dependant on scalar type are defined in this templatized class");

			pyPointMatcherIO.def(py::init<>())
				.def_static("getColLabel", &PMIO::getColLabel, py::arg("label"), py::arg("row"), "convert a descriptor label to an appropriate sub-label");

			using PMPropTypes = PMIO::PMPropTypes;
			py::enum_<PMPropTypes>(pyPointMatcherIO, "PMPropTypes", "Type of information in a DataPoints. Each type is stored in its own dense matrix.")
				.value("FEATURE", PMPropTypes::FEATURE).value("DESCRIPTOR", PMPropTypes::DESCRIPTOR)
				.value("TIME", PMPropTypes::TIME).value("UNSUPPORTED", PMPropTypes::UNSUPPORTED);

			using SupportedLabel = PMIO::SupportedLabel;
			py::class_<SupportedLabel>(pyPointMatcherIO, "SupportedLabel", "Structure containing all information required to map external information to PointMatcher internal representation")
				.def_readwrite("internalName", &SupportedLabel::internalName, "name used in PointMatcher")
				.def_readwrite("externalName", &SupportedLabel::externalName, "name used in external format")
				.def_readwrite("type", &SupportedLabel::type, "type of information in PointMatcher")

				.def(py::init<const std::string&, const std::string&, const PMPropTypes&>(), py::arg("internalName"), py::arg("externalName"), py::arg("type"));

			using SupportedLabels = PMIO::SupportedLabels;
			py::bind_vector<SupportedLabels>(pyPointMatcherIO, "SupportedLabels", "Vector of supported labels in PointMatcher and their external names");

			using GenericInputHeader = PMIO::GenericInputHeader;
			py::class_<GenericInputHeader>(pyPointMatcherIO, "GenericInputHeader", "Helper structure designed to parse file headers")
				.def_readwrite("name", &GenericInputHeader::name)
				.def_readwrite("matrixRowId", &GenericInputHeader::matrixRowId)
				.def_readwrite("matrixType", &GenericInputHeader::matrixType).def(py::init<>())
				.def(py::init<const std::string&>(), py::arg("name"));

			pyPointMatcherIO.def_static("getSupportedExternalLabels", &PMIO::getSupportedExternalLabels, R"pbdoc(
Vector containing the mapping of all external names to PointMatcher representation.
The order is important (i.e., nx before ny). This can also be used to remap 1D descriptor name to a better one.
)pbdoc");

			using LabelGenerator = PMIO::LabelGenerator;
			py::class_<LabelGenerator>(pyPointMatcherIO, "LabelGenerator").def(py::init<>())
				.def("add", (void (LabelGenerator::*)(const std::string)) &LabelGenerator::add, py::arg("internalName"), "add a name to the vector of labels. If already there, will increament the dimension.")
				.def("add", (void (LabelGenerator::*)(const std::string, const unsigned int)) &LabelGenerator::add, py::arg("internalName"), py::arg("dim"), "add a name to the vector of labels with its dimension.")
				.def("getLabels", &LabelGenerator::getLabels, "Return the vector of labels used to build a DataPoints");

			pyPointMatcherIO
				.def_static("loadCSV", (DataPoints (*)(const std::string&)) &PMIO::loadCSV, py::arg("fileName"))
				.def_static("saveCSV", (void (*)(const DataPoints&, const std::string&)) &PMIO::saveCSV, py::arg("data"), py::arg("fileName"));

			using SupportedVTKDataTypes = PMIO::SupportedVTKDataTypes;
			py::enum_<SupportedVTKDataTypes>(pyPointMatcherIO, "SupportedVTKDataTypes", "Enumeration of legacy VTK data types that can be parsed")
				.value("POLYDATA", SupportedVTKDataTypes::POLYDATA)
				.value("UNSTRUCTURED_GRID", SupportedVTKDataTypes::UNSTRUCTURED_GRID);

			using SplitTime = PMIO::SplitTime;
			py::class_<SplitTime>(pyPointMatcherIO, "SplitTime", "Storage for time loaded separatly")
				.def_readwrite("isHigh32Found", &SplitTime::isHigh32Found, "was the high 32bits found in the file")
				.def_readwrite("isLow32Found", &SplitTime::isLow32Found, "was the low 32bits found in the file")
				.def_readwrite("high32", &SplitTime::high32, "Matrix containing file data representing the high 32 bits")
				.def_readwrite("low32", &SplitTime::low32, "Matrix containing file data representing the low 32 bits")
				.def(py::init<>());

			pyPointMatcherIO
				.def_static("loadVTK", (DataPoints (*)(const std::string&)) &PMIO::loadVTK, py::arg("fileName"))
				.def_static("saveVTK", (void (*)(const DataPoints&, const std::string&, bool)) &PMIO::saveVTK, py::arg("data"), py::arg("fileName"), py::arg("binary") = false)

				.def_static("loadPLY", (DataPoints (*)(const std::string&)) &PMIO::loadPLY, py::arg("fileName"))
				.def_static("savePLY", (void (*)(const DataPoints&, const std::string&)) &PMIO::savePLY, py::arg("data"), py::arg("fileName"), "save datapoints to PLY point cloud format")

				.def_static("loadPCD", (DataPoints (*)(const std::string&)) &PMIO::loadPCD, py::arg("fileName"))
				.def_static("savePCD", (void (*)(const DataPoints&, const std::string&)) &PMIO::savePCD, py::arg("data"), py::arg("fileName"), "save datapoints to PCD point cloud format");

			using FileInfo = PMIO::FileInfo;
			using Vector3 = FileInfo::Vector3;
			py::class_<FileInfo>(pyPointMatcherIO, "FileInfo", "Information to exploit a reading from a file using this library. Fields might be left blank if unused.")
				.def_readwrite("readingFileName", &FileInfo::readingFileName, "file name of the reading point cloud")
				.def_readwrite("referenceFileName", &FileInfo::referenceFileName, "file name of the reference point cloud")
				.def_readwrite("configFileName", &FileInfo::configFileName, "file name of the yaml configuration")
				.def_readwrite("initialTransformation", &FileInfo::initialTransformation, "matrix of initial estimate transform")
				.def_readwrite("groundTruthTransformation", &FileInfo::groundTruthTransformation, "matrix of the ground-truth transform")
				.def_readwrite("gravity", &FileInfo::gravity, "gravity vector")

				.def(py::init<const std::string&, const std::string&, const std::string&, const TransformationParameters&, const TransformationParameters&, const Vector&>(), py::arg("readingPath") = "", py::arg(" referencePath") = "", py::arg("configFileName") = "", py::arg("initialTransformation") = TransformationParameters(), py::arg("groundTruthTransformation") = TransformationParameters(), py::arg("gravity") = Vector3::Zero(), "Constructor, leave fields blank if unused");

			using FileInfoVector = PMIO::FileInfoVector;
			py::bind_vector<FileInfoVector>(pyPointMatcherIO, "FileInfoVector", "A vector of file info, to be used in batch processing")
				.def(py::init<>(), "Empty constructor")
				.def(py::init<const std::string&, std::string, std::string>(), py::arg("fileName"), py::arg("dataPath") = "", py::arg("configPath") = "", R"pbdoc(
Load a vector of FileInfo from a CSV file.

@param fileName name of the CSV file
@param dataPath path relative to which the point cloud CSV or VTK will be resolved
@param configPath path relative to which the yaml configuration files will be resolved

The first line of the CSV file must contain a header. The supported tags are:
- reading: file name of the reading point cloud
- reference: file name of the reference point cloud
- config: file name of the YAML configuration of the ICP chain
- iTxy: initial transformation, coordinate x,y
- gTxy: ground-truth transformation, coordinate x,y
Note that the header must at least contain "reading".
)pbdoc");

			using CsvDescriptor = PMIO::CsvDescriptor;
			py::class_<CsvDescriptor>(pyPointMatcherIO, "CsvDescriptor", "A structure to hold information about descriptors contained in a CSV file")
				.def(py::init<>()).def_readwrite("name", &CsvDescriptor::name, "name of descriptor")
				.def_readwrite("start_col", &CsvDescriptor::start_col, "column number at which descriptor starts")
				.def_readwrite("span", &CsvDescriptor::span, "number of columns spanned by descriptor");

			pyPointMatcherIO
				.def_static("plyPropTypeValid", &PMIO::plyPropTypeValid, "Check that property defined by type is a valid PLY type note: type must be lowercase");

			using PLYProperty = PMIO::PLYProperty;
			py::class_<PLYProperty>(pyPointMatcherIO, "PLYProperty", "Interface for PLY property")
				.def_readwrite("name", &PLYProperty::name, "name of PLY property")
				.def_readwrite("type", &PLYProperty::type, "type of PLY property")
				.def_readwrite("idx_type", &PLYProperty::idx_type, "for list properties, type of number of elements")
				.def_readwrite("pos", &PLYProperty::pos, "index of the property in element")
				.def_readwrite("is_list", &PLYProperty::is_list, "member is true of property is a list")
				.def_readwrite("pmType", &PLYProperty::pmType, "type of information in PointMatcher")
				.def_readwrite("pmRowID", &PLYProperty::pmRowID, "row id used in a DataPoints")

				.def(py::init<>(), "Default constructor. If used member values must be filled later.")
				.def(py::init<const std::string&, const std::string&, const unsigned>(), py::arg("type"), py::arg("name"), py::arg("pos"), "regular property")
				.def(py::init<const std::string&, const std::string&, const std::string&, const unsigned>(), py::arg("idx_type"), py::arg("type"), py::arg("name"), py::arg("pos"), "list property")

				.def("__eq__", &PLYProperty::operator==, "compare with other property");

			using PLYProperties = PMIO::PLYProperties;
			py::bind_vector<PLYProperties>(pyPointMatcherIO, "PLYProperties", "Vector of properties specific to PLY files");

			using PLYDescPropMap = PMIO::PLYDescPropMap;
			py::bind_map<PLYDescPropMap>(pyPointMatcherIO, "PLYDescPropMap", "Map from a descriptor name to a list PLY property\nex: \"normals\" -> nx, ny ,nz");

			using PLYElement = PMIO::PLYElement;
			py::class_<PLYElement>(pyPointMatcherIO, "PLYElement", "Interface for all PLY elements.")
				.def_readwrite("name", &PLYElement::name, "name identifying the PLY element")
				.def_readwrite("num", &PLYElement::num, "number of occurences of the element")
				.def_readwrite("total_props", &PLYElement::total_props, "total number of properties in PLY element")
				.def_readwrite("offset", &PLYElement::offset, "line at which data starts")
				.def_readwrite("properties", &PLYElement::properties, "all properties found in the header")
				.def_readwrite("nbFeatures", &PLYElement::nbFeatures, "number of valid features found in the header")
				.def_readwrite("nbDescriptors", &PLYElement::nbDescriptors, "number of valid descriptors found in the header")

				.def(py::init<const std::string&, const unsigned, const unsigned>(), py::arg("name"), py::arg("num"), py::arg("offset"), R"pbdoc(
PLY Element constructor

@param name name of the ply element (case-sensitive)
@param num number of times the element appears in the file
@param offset if there are several elements, the line offset at which this element begins.  Note that, as of writing, only one (vertex) element is supported.

This object holds information about a PLY element contained in the file.
It is filled out when reading the header and used when parsing the data.
)pbdoc").def("__eq__", &PLYElement::operator==, "comparison operator for elements");

			using PLYVertex = PMIO::PLYVertex;
			py::class_<PLYVertex, PLYElement>(pyPointMatcherIO, "PLYVertex", "Implementation of PLY vertex element")
				.def(py::init<const unsigned, const unsigned>(), py::arg("num"), py::arg("offset"), R"pbdoc(
Constructor

@param num number of times the element appears in the file
@param offset if there are several elements, the line offset at which this element begins.  Note that, as of writing, only one (vertex) element is supported.

Implementation of PLY element interface for the vertex element
)pbdoc");

//		FIXME : Generate undefined symbol error for "elementSupported" or "createElement" method when importing the module
//		using PLYElementF = PMIO::PLYElementF;
//		py::class_<PLYElementF>(pyPointMatcherIO, "PLYElementF", "Factory for PLY elements")
//			.def("elementSupported", &PLYElementF::elementSupported, py::arg("elem_name"), "returns true if element named elem_name is supported by this parser")
//			.def_static("createElement", &PLYElementF::createElement, py::arg("elem_name"), py::arg("elem_num"), py::arg("offset"), "factory function, build element defined by name with elem_num elements");

			using PCDproperty = PMIO::PCDproperty;
			py::class_<PCDproperty>(pyPointMatcherIO, "PCDproperty", "Information for a PCD property")
				.def_readwrite("field", &PCDproperty::field, "Name of the property")
				.def_readwrite("size", &PCDproperty::size, "Size of the property in bytes")
				.def_readwrite("type", &PCDproperty::type, "Type: I: signed, U: unsigned, F: float")
				.def_readwrite("count", &PCDproperty::count, "number of dimension")
				.def_readwrite("pmType", &PCDproperty::pmType, "type of information in PointMatcher")
				.def_readwrite("pmRowID", &PCDproperty::pmRowID, "row id used in a DataPoints")

				.def(py::init<>());

			using PCDheader = PMIO::PCDheader;
			py::class_<PCDheader>(pyPointMatcherIO, "PCDheader", "All information contained in the header of a PCD file")
				.def_readwrite("version", &PCDheader::version, "Version of the PCD file used")
				.def_readwrite("properties", &PCDheader::properties, "vector of properties")
				.def_readwrite("width", &PCDheader::width, "width of sensor matrix")
				.def_readwrite("height", &PCDheader::height, "height of sensor matrix")
				.def_readwrite("viewPoint", &PCDheader::viewPoint, "not used")
				.def_readwrite("nbPoints", &PCDheader::nbPoints, "number of points, same as width*height")
				.def_readwrite("dataType", &PCDheader::dataType, "ascii or binary")

				.def(py::init<>());
		}
	}
}
