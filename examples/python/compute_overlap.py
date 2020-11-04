import numpy as np

from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms

# Code example for DataFilter taking a sequence of point clouds with
# their global coordinates and build a map with a fix (manageable) number of points

PM = pm.PointMatcher
PMIO = pm.PointMatcherIO
DP = PM.DataPoints
Matches = PM.Matches
params = pms.Parametrizable.Parameters()

# Path of output directory (default: tests/compute_overlap/)
# The output directory must already exist
# Leave empty to save in the current directory
output_base_directory = "tests/compute_overlap/"

# Name of output files (default: test)
output_base_file = "test"

# Loading the list of files
file_info_list = PMIO.FileInfoVector("../data/carCloudList.csv", "../data/")
# or
# file_info_vec = PMIO.FileInfoVector("../data/cloudList.csv", "../data/")

# If True, it will compute the overlap of only 2 point cloud ids
# and dump VTK files for visual inspection
debug_mode = False

# Choose the first point cloud to compute the overlap and debug.
# Must be different than debug_J
debug_I = 1

# Choose the second point cloud to compute the overlap and debug
# Must be different than debug_I
debug_J = 0

if debug_mode:
    pms.setLogger(PM.get().LoggerRegistrar.create("FileLogger"))

# Prepare transformation chain for maps
rigid_trans = PM.get().TransformationRegistrar.create("RigidTransformation")

transformations = PM.Transformations()
transformations.append(rigid_trans)

Tread = np.identity(4)
Tref = np.identity(4)

starting_I = 0
list_size_I = len(file_info_list)
list_size_J = len(file_info_list)

overlap_results = np.ones((list_size_J, list_size_I), np.float)

if debug_mode:
    starting_I = debug_I
    list_size_I = starting_I + 1

for i in range(starting_I, list_size_I):
    starting_J = i + 1

    if debug_mode:
        starting_J = debug_J
        list_size_J = starting_J + 1

    for j in range(starting_J, list_size_J):
        #  Load point clouds
        reading = DP.load(file_info_list[i].readingFileName)
        reference = DP.load(file_info_list[j].readingFileName)

        print("Point cloud loaded")

        #  Load transformation matrices
        if file_info_list[i].groundTruthTransformation.shape[0] != 0:
            Tread = file_info_list[i].groundTruthTransformation
            Tref = file_info_list[j].groundTruthTransformation
        else:
            print("ERROR: fields gTXX (i.e., ground truth matrix) is required")
            exit()

        #  Move point cloud in global frame
        transformations.apply(reading, Tread)
        transformations.apply(reference, Tref)

        #  Prepare filters
        params["prob"] = "0.5"
        sub_sample = PM.get().DataPointsFilterRegistrar.create("RandomSamplingDataPointsFilter",
                                                               params)
        params.clear()

        max_density = PM.get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter")

        # params["dim"] = "1"
        # params["minDist"] = "0"
        # cut_in_half = PM.get().DataPointsFilterRegistrar.create("MinDistDataPointsFilter",
        #                                                         params)
        # params.clear()

        params["knn"] = "20"
        params["keepDensities"] = "1"
        compute_density = PM.get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter",
                                                                    params)
        params.clear()

        reading = sub_sample.filter(reading)
        reading = compute_density.filter(reading)
        reading = max_density.filter(reading)
        # reading = cut_in_half.filter(reading)
        inliers_read = np.zeros((1, reading.features.shape[1]))
        reading.addDescriptor("inliers", inliers_read)

        reference = sub_sample.filter(reference)
        reference = compute_density.filter(reference)
        reference = max_density.filter(reference)
        inliers_ref = np.zeros((1, reference.features.shape[1]))
        reference.addDescriptor("inliers", inliers_ref)

        self = reading
        target = reference

        for l in range(2):
            self_pts_count = self.features.shape[1]
            target_pts_count = target.features.shape[1]

            #  Build kd-tree
            knn = 20
            knn_all = 50

            params["knn"] = str(knn)
            matcher_self = PM.get().MatcherRegistrar.create("KDTreeMatcher", params)
            params.clear()

            params["knn"] = str(knn_all)
            params["maxDistField"] = "maxSearchDist"
            matcher_target = PM.get().MatcherRegistrar.create("KDTreeVarDistMatcher", params)
            params.clear()

            matcher_self.init(self)
            matcher_target.init(target)

            self_matches = Matches(knn, self_pts_count)
            self_matches = matcher_self.findClosests(self)

            max_search_dist = np.sqrt(self_matches.dists.max(axis=0, keepdims=True), order='F')
            self.addDescriptor("maxSearchDist", max_search_dist)

            target_matches = Matches(knn_all, target_pts_count)
            target_matches = matcher_target.findClosests(self)

            inlier_self = self.getDescriptorViewByName("inliers")
            inlier_target = target.getDescriptorViewByName("inliers")

            for m in range(self_pts_count):
                for n in range(knn_all):
                    if target_matches.dists[n, m] != np.infty:
                        inlier_self[0, m] = 1.0
                        inlier_target[0, target_matches.ids[n, m]] = 1.0

            PM.get().swapDataPoints(self, target)

        final_inlier_self = self.getDescriptorViewByName("inliers")
        final_inlier_target = target.getDescriptorViewByName("inliers")
        self_ratio = np.count_nonzero(final_inlier_self) / final_inlier_self.shape[1]
        target_ratio = np.count_nonzero(final_inlier_target) / final_inlier_target.shape[1]

        print(f"{i} -> {j}: {self_ratio:.6}")
        print(f"{j} -> {i}: {target_ratio:.6}")

        overlap_results[j, i] = self_ratio
        overlap_results[i, j] = target_ratio

        if debug_mode:
            self.save(f"{output_base_directory + output_base_file}_scan_i.vtk")
            target.save(f"{output_base_directory + output_base_file}_scan_j.vtk")

with open(f"{output_base_directory}overlap_results.csv", 'w') as out_file:
    for x in range(overlap_results.shape[0]):
        for y in range(overlap_results.shape[1]):
            out_file.write(f"{overlap_results[x, y]:.6}, ")

        out_file.write('\n')
