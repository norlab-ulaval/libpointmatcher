from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms

PM = pm.PointMatcher
DP = PM.DataPoints
output_base_file = "tests_output/icp_customized/"
is_3D = True  # (toggle to switch between 2D and 3D clouds)

# Load point clouds
if is_3D:
    # 3D point clouds
    ref = DP.load('../data/car_cloud400.csv')
    data = DP.load('../data/car_cloud401.csv')
    test_base = "3D"
else:
    # 2D point clouds
    ref = DP.load('../data/2D_twoBoxes.csv')
    data = DP.load('../data/2D_oneBox.csv')
    test_base = "2D"

# Create the default ICP algrotithm
icp = PM.ICP()

pms.setLogger(PM.get().LoggerRegistrar.create("FileLogger"))

# Prepare reading filters
name = "MinDistDataPointsFilter"
params = {"minDist": "1.0"}
minDist_read = PM.get().DataPointsFilterRegistrar.create(name, params)

name = "RandomSamplingDataPointsFilter"
params = {"prob": "0.05"}
rand_read = PM.get().DataPointsFilterRegistrar.create(name, params)

# Prepare reference filters
name = "MinDistDataPointsFilter"
params = {"minDist": "1.0"}
minDist_ref = PM.get().DataPointsFilterRegistrar.create(name, params)

name = "RandomSamplingDataPointsFilter"
params = {"prob": "0.05"}
rand_ref = PM.get().DataPointsFilterRegistrar.create(name, params)

# Prepare matching function
name = "KDTreeMatcher"
params = {"knn": "1", "epsilon": "3.16"}
kdtree = PM.get().MatcherRegistrar.create(name, params)

# Prepare outlier filters
name = "TrimmedDistOutlierFilter"
params = {"ratio": "0.75"}
trim = PM.get().OutlierFilterRegistrar.create(name, params)

# Prepare error minimization
name = "PointToPointErrorMinimizer"
pointToPoint = PM.get().ErrorMinimizerRegistrar.create(name)

# Prepare transformation checker filters
name = "CounterTransformationChecker"
params = {"maxIterationCount": "150"}
maxIter = PM.get().TransformationCheckerRegistrar.create(name, params)

name = "DifferentialTransformationChecker"
params = {"minDiffRotErr": "0.001", "minDiffTransErr": "0.01", "smoothLength": "4"}
diff = PM.get().TransformationCheckerRegistrar.create(name, params)

# Prepare inspector
# toggle to write vtk files per iteration
name = "NullInspector"
nullInspect = PM.get().InspectorRegistrar.create(name)

# uncomment to write vtk files per iteration
# name = "VTKFileInspector"
# params["dumpDataLinks"] = "1"
# params["dumpReading"] = "1"
# params["dumpReference"] = "1"
# vtkInspect = PM.get().InspectorRegistrar.create(name, params)
# params = Parametrizable.Parameters()

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

# toggle to write vtk files per iteration
icp.inspector = nullInspect
# icp.inspector = vtkInspect

icp.transformations.append(rigid_trans)

T = icp(data, ref)

data_out = DP(data)

icp.transformations.apply(data_out, T)

ref.save(output_base_file + f"{test_base}_test_ref.vtk")
data.save(output_base_file + f"{test_base}_test_data_in.vtk")
data_out.save(output_base_file + f"{test_base}_test_data_out.vtk")

print(f"{test_base} ICP transformation:\n{T}".replace("[", " ").replace("]", " "))
