#ifndef PYTHON_PYPOINT_MATCHER_HELPER_H
#define PYTHON_PYPOINT_MATCHER_HELPER_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "pointmatcher/PointMatcher.h"

namespace py = pybind11;
namespace pms = PointMatcherSupport;

// PointMatcher aliases
using PM = PointMatcher<float>;
using DataPoints = PM::DataPoints;
using Label = DataPoints::Label;
using Labels = DataPoints::Labels;
using Matches = PM::Matches;
using Transformation = PM::Transformation;
using Transformations = PM::Transformations;
using DataPointsFilter = PM::DataPointsFilter;
using DataPointsFilters = PM::DataPointsFilters;
using Matcher = PM::Matcher;
using OutlierFilter = PM::OutlierFilter;
using OutlierFilters = PM::OutlierFilters;
using ErrorMinimizer = PM::ErrorMinimizer;
using ErrorElements = ErrorMinimizer::ErrorElements;
using TransformationChecker = PM::TransformationChecker;
using TransformationCheckers = PM::TransformationCheckers;
using Inspector = PM::Inspector;
using ICPChaineBase = PM::ICPChainBase;
using ICP = PM::ICP;
using ICPSequence = PM::ICPSequence;

// PointMatcherSupport aliases
using Parametrizable = pms::Parametrizable;
using Parameter = Parametrizable::Parameter;
using Parameters = Parametrizable::Parameters;
using ParameterDoc = Parametrizable::ParameterDoc;
using ParametersDoc = Parametrizable::ParametersDoc;
using InvalidParameter = Parametrizable::InvalidParameter;
using Logger = pms::Logger;
using DataPointsFilterRegistrar = pms::Registrar<DataPointsFilter>;
using ErrorMinimizerRegistrar = pms::Registrar<ErrorMinimizer>;
using InspectorRegistrar = pms::Registrar<Inspector>;
using LoggerRegistrar = pms::Registrar<Logger>;
using MatcherRegistrar = pms::Registrar<Matcher>;
using OutlierFilterRegistrar = pms::Registrar<OutlierFilter>;
using TransformationRegistrar = pms::Registrar<Transformation>;
using TransformationCheckerRegistrar = pms::Registrar<TransformationChecker>;

// Eigen and nabo-based types aliases
using ScalarType = PM::ScalarType;
using Vector = PM::Vector;
using VectorVector = PM::VectorVector;
using Quaternion = PM::Quaternion;
using QuaternionVector = PM::QuaternionVector;
using Matrix = PM::Matrix;
using IntMatrix = PM::IntMatrix;
using Int64Matrix = PM::Int64Matrix;
using Array = PM::Array;
using TransformationParameters = PM::TransformationParameters;
using OutlierWeights = PM::OutlierWeights;

PYBIND11_MAKE_OPAQUE(std::vector<std::string>) // StringVector
PYBIND11_MAKE_OPAQUE(std::map<std::string, std::map<std::string, std::string>>) // Bibliography
PYBIND11_MAKE_OPAQUE(std::map<std::string, unsigned>) // BibIndices
PYBIND11_MAKE_OPAQUE(std::vector<ParameterDoc>) // ParametersDoc
PYBIND11_MAKE_OPAQUE(std::map<std::string, std::string>) // Parameters, StringMap
PYBIND11_MAKE_OPAQUE(std::map<std::string, std::vector<std::string>>) // CsvElements
PYBIND11_MAKE_OPAQUE(std::vector<Label>) // Labels
PYBIND11_MAKE_OPAQUE(std::vector<DataPointsFilter>) // DataPointsFilters
PYBIND11_MAKE_OPAQUE(std::vector<Transformation>) // Transformations
PYBIND11_MAKE_OPAQUE(std::vector<OutlierFilter>) // OutlierFilters
PYBIND11_MAKE_OPAQUE(std::vector<TransformationChecker>) // TransformationCheckers

#endif //PYTHON_PYPOINT_MATCHER_HELPER_H
