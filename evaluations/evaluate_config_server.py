#!/usr/bin/env python3

import argparse
import json
import os
import sys

import numpy as np
from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms
from scipy.spatial.transform import Rotation

PM = pm.PointMatcher
DP = PM.DataPoints
Parameters = pms.Parametrizable.Parameters


def get_angle_error(P: np.array, Q: np.array) -> float:
    R = np.dot(P, Q.T)
    theta = (np.trace(R) - 1) / 2
    theta = min(theta, 1.0)
    theta = max(theta, -1.0)
    return np.arccos(theta)


def build_tf_matrix(rotation: np.array, translation: np.array) -> np.array:
    tf = np.identity(4)
    tf[0:3, 0:3] = rotation
    tf[0:3, 3] = translation
    return tf


def get_random_translation(num=None, seed=None):
    if num is None:
        num = 1
    np.random.seed(seed)
    return 2.0*(np.random.rand(3, num)-0.5)


def main(config_file: str, path: str, output: str, seed: int, number_of_random_transforms=2):
    if not os.path.exists(config_file):
        raise FileNotFoundError("The specified config file does not exist: {}".format(config_file))
    if not os.path.exists(path):
        raise FileNotFoundError("The specified point-cloud path does not exist: {}".format(path))
    if not os.path.exists(os.path.dirname(output)):
        raise FileNotFoundError(f"The output directory does not exist: {os.path.dirname(output)}")
    if os.path.isfile(output):
        raise FileExistsError(f"The output file already exists: {output}")

    # List all files in path/easy, path/medium, path/hard
    paths_dict = {'easy': os.path.join(path, "easy"),
                  'medium': os.path.join(path, "medium"),
                  'hard': os.path.join(path, "hard")
                  }

    # Create ICP and load YAML config
    icp = PM.ICP()
    icp.loadFromYaml(config_file)

    results_dict = {}

    for difficulty, path in paths_dict.items():
        results_dict[difficulty] = {}
        # iterate over all files in the difficulty folder
        for (dirpath, dirnames, filenames) in os.walk(path):
            for filename in filenames:
                results_dict[difficulty][filename] = {}

                point_cloud_path = os.path.join(path, filename)
                print(f"Processing {point_cloud_path}")
                # load the reference point cloud to libpointmatcher
                reference = DP(DP.load(point_cloud_path))
                # generate N random transformations
                rotations = Rotation.random(number_of_random_transforms, seed).as_matrix()
                translations = get_random_translation(number_of_random_transforms, seed)
                seed += number_of_random_transforms

                errors_rot = []
                errors_trans = []
                transformations = []
                for i in range(number_of_random_transforms):
                    tf = build_tf_matrix(rotations[i], translations[:, i])

                    print("tf:\n", tf)
                    # apply the transformation
                    transformation = PM.get().TransformationRegistrar.create("RigidTransformation")
                    reading = transformation.compute(reference, tf)

                    # register
                    tf_icp = icp(reading, reference)

                    # Compute error in translation and rotation
                    error_rot = float(np.linalg.norm(tf_icp[0:3, 3]))
                    error_trans = get_angle_error(tf_icp[0:3, 0:3], np.identity(3))

                    # Save the errors
                    errors_rot.append(error_rot)
                    errors_trans.append(error_trans)
                    transformations.append(tf_icp.tolist())

                results_dict[difficulty][filename]["error_translation"] = errors_trans
                results_dict[difficulty][filename]["error_rotation"] = errors_rot
                results_dict[difficulty][filename]["transformations"] = transformations

    # Writing to the output .json file
    with open(output, 'w', encoding='utf-8') as f:
        json.dump(results_dict, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Dummy script for libpointmatcher-server evaluation.'
                    'Use with absolute paths.'
                    'Your pipeline should accommodate output files with possible multiple data lines.')
    parser.add_argument('--config', type=str, required=True,
                        help='Path to yaml configuration file')
    parser.add_argument('--path', type=str, required=True,
                        help='Path to a folder containing evaluation point clouds.'
                             'The clouds are organized into three folders: easy, medium and hard')
    parser.add_argument('--output', type=str, required=True,
                        help='Output path with score values')
    parser.add_argument('--seed', type=int, required=True,
                        help='Seed value')

    args = parser.parse_args()
    try:
        main(args.config, args.path, args.output, args.seed)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    sys.exit(0)