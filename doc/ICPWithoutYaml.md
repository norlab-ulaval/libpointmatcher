| [Tutorials Home](index.md) | [Previous](Transformations.md) | [Next](DataPointsFilterDev.md) |
| :--- | :---: | ---: |

# Example of an ICP solution without yaml

See [examples/icp_customized.cpp](https://github.com/ethz-asl/libpointmatcher/blob/master/examples/icp_customized.cpp) for a working example.

Here are the important part of the example. First, generate an empty ICP object with some generic variables:

```cpp
// Create the default ICP algorithm
PM::ICP icp;
PointMatcherSupport::Parametrizable::Parameters params;
std::string name;
```

Prepare the objects for the DataFilters:

```cpp
// Prepare reading filters
name = "MinDistDataPointsFilter";
params["minDist"] = "1.0";
std::shared_ptr<PM::DataPointsFilter> minDist_read = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();

name = "RandomSamplingDataPointsFilter";
params["prob"] = "0.05";
std::shared_ptr<PM::DataPointsFilter> rand_read = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();

// Prepare reference filters
name = "MinDistDataPointsFilter";
params["minDist"] = "1.0";
std::shared_ptr<PM::DataPointsFilter> minDist_ref = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();

name = "RandomSamplingDataPointsFilter";
params["prob"] = "0.05";
std::shared_ptr<PM::DataPointsFilter> rand_ref = 
	PM::get().DataPointsFilterRegistrar.create(name, params);
params.clear();
```

Prepare the objects for the Matchers:

```cpp
// Prepare matching function
name = "KDTreeMatcher";
params["knn"] = "1";
params["epsilon"] = "3.16";
std::shared_ptr<PM::Matcher> kdtree = 
	PM::get().MatcherRegistrar.create(name, params);
params.clear();
```

Prepare the objects for the OutlierFilters:

```cpp
// Prepare outlier filters
name = "TrimmedDistOutlierFilter";
params["ratio"] = "0.75";
std::shared_ptr<PM::OutlierFilter> trim = 
	PM::get().OutlierFilterRegistrar.create(name, params);
params.clear();
```

Prepare the object for the ErrorMinimizer:

```cpp
// Prepare error minimization
name = "PointToPointErrorMinimizer";
std::shared_ptr<PM::ErrorMinimizer> pointToPoint =   
	PM::get().ErrorMinimizerRegistrar.create(name);
```

Prepare the objects for the TransformationCheckers:

```cpp
// Prepare outlier filters
name = "CounterTransformationChecker";
params["maxIterationCount"] = "150";
std::shared_ptr<PM::TransformationChecker> maxIter = 
	PM::get().TransformationCheckerRegistrar.create(name, params);
params.clear();

name = "DifferentialTransformationChecker";
params["minDiffRotErr"] = "0.001";
params["minDiffTransErr"] = "0.01";
params["smoothLength"] = "4";
std::shared_ptr<PM::TransformationChecker> diff = 
	PM::get().TransformationCheckerRegistrar.create(name, params);
params.clear();
```

Prepare the objects for the Inspector:

```cpp
// Prepare inspector
std::shared_ptr<PM::Inspector> nullInspect =
	PM::get().InspectorRegistrar.create("NullInspector");
```

Prepare the objects for the Transformation:

```cpp
// Prepare transformation
std::shared_ptr<PM::Transformation> rigidTrans =
	PM::get().TransformationRegistrar.create("RigidTransformation");
```

Finally, build the complete solution:

```cpp
// Build ICP solution
icp.readingDataPointsFilters.push_back(minDist_read);
icp.readingDataPointsFilters.push_back(rand_read);

icp.referenceDataPointsFilters.push_back(minDist_ref);
icp.referenceDataPointsFilters.push_back(rand_ref);

icp.matcher = kdtree;

icp.outlierFilters.push_back(trim);

icp.errorMinimizer = pointToPoint;

icp.transformationCheckers.push_back(maxIter);
icp.transformationCheckers.push_back(diff);

// toggle to write vtk files per iteration
icp.inspector = nullInspect;
//icp.inspector = vtkInspect;

icp.transformations.push_back(rigidTrans);
```
