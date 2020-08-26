# Code example for ICP taking 2 points clouds (2D or 3D) relatively close
# and computing the transformation between them.
#
# Instead of using yaml file for configuration, we configure the solution
# directly in the code.
#
# This code replicate the solution in /evaluations/official_solutions/Besl92_pt2point.yaml

from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms

PM = pm.PointMatcher
DP = PM.DataPoints
Parameters = pms.Parametrizable.Parameters

# Path of output directory (default: tests/icp_customized/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/icp_customized/"

# Name of output files (default: test)
output_base_file = "test"

# Toggle to switch between 2D and 3D clouds
is_3D = True

if is_3D:
    # Load 3D point clouds
    ref = DP(DP.load('../data/car_cloud400.csv'))
    data = DP(DP.load('../data/car_cloud401.csv'))
    test_base = "3D"
else:
    # Load 2D point clouds
    ref = DP(DP.load('../data/2D_twoBoxes.csv'))
    data = DP(DP.load('../data/2D_oneBox.csv'))
    test_base = "2D"

# Create the default ICP algrotithm
icp = PM.ICP()
params = Parameters()

# Comment out to stop console outputs
pms.setLogger(PM.get().LoggerRegistrar.create("FileLogger"))

# Prepare reading filters
name = "MinDistDataPointsFilter"
params["minDist"] = "1.0"
minDist_read = PM.get().DataPointsFilterRegistrar.create(name, params)
params.clear()

name = "RandomSamplingDataPointsFilter"
params["prob"] = "0.05"
rand_read = PM.get().DataPointsFilterRegistrar.create(name, params)
params.clear()

# Prepare reference filters
name = "MinDistDataPointsFilter"
params["minDist"] = "1.0"
minDist_ref = PM.get().DataPointsFilterRegistrar.create(name, params)
params.clear()

name = "RandomSamplingDataPointsFilter"
params["prob"] = "0.05"
rand_ref = PM.get().DataPointsFilterRegistrar.create(name, params)
params.clear()

# Prepare matching function
name = "KDTreeMatcher"
params["knn"] = "1"
params["epsilon"] = "3.16"
kdtree = PM.get().MatcherRegistrar.create(name, params)
params.clear()

# Prepare outlier filters
name = "TrimmedDistOutlierFilter"
params["ratio"] = "0.75"
trim = PM.get().OutlierFilterRegistrar.create(name, params)
params.clear()

# Prepare error minimization
name = "PointToPointErrorMinimizer"
pointToPoint = PM.get().ErrorMinimizerRegistrar.create(name)
params.clear()

# Prepare transformation checker filters
name = "CounterTransformationChecker"
params["maxIterationCount"] = "150"
maxIter = PM.get().TransformationCheckerRegistrar.create(name, params)
params.clear()

name = "DifferentialTransformationChecker"
params["minDiffRotErr"] = "0.001"
params["minDiffTransErr"] = "0.01"
params["smoothLength"] = "4"
diff = PM.get().TransformationCheckerRegistrar.create(name, params)
params.clear()

# Prepare inspector
# Comment out to write vtk files per iteration
name = "NullInspector"
nullInspect = PM.get().InspectorRegistrar.create(name)

# Uncomment to write vtk files per iteration
# name = "VTKFileInspector"
# params["dumpDataLinks"] = "1"
# params["dumpReading"] = "1"
# params["dumpReference"] = "1"
# vtkInspect = PM.get().InspectorRegistrar.create(name, params)
# params.clear()

# Prepare transformation
name = "RigidTransformation"
rigid_trans = PM.get().TransformationRegistrar.create(name)

# Build ICP solution
icp.readingDataPointsFilters.append(minDist_read)
icp.readingDataPointsFilters.append(rand_read)

icp.referenceDataPointsFilters.append(minDist_ref)
icp.referenceDataPointsFilters.append(rand_ref)

icp.matcher = kdtree

icp.outlierFilters.append(trim)

icp.errorMinimizer = pointToPoint

icp.transformationCheckers.append(maxIter)
icp.transformationCheckers.append(diff)

# Toggle to write vtk files per iteration
icp.inspector = nullInspect
# icp.inspector = vtkInspect

icp.transformations.append(rigid_trans)

# Compute the transformation to express data in ref
T = icp(data, ref)

# Transform data to express it in ref
data_out = DP(data)

icp.transformations.apply(data_out, T)

# Save files to see the results
ref.save(f"{output_base_directory + test_base}_{output_base_file}_ref.vtk")
data.save(f"{output_base_directory + test_base}_{output_base_file}_data_in.vtk")
data_out.save(f"{output_base_directory + test_base}_{output_base_file}_data_out.vtk")

print(f"{test_base} ICP transformation:\n{T}".replace("[", " ").replace("]", " "))
