#!/usr/bin/env python3

import argparse
import asyncio
import json
import os
import sys
import base64
import uuid
from websockets import serve, WebSocketServerProtocol

import numpy as np
from pypointmatcher import pointmatcher as pm, pointmatchersupport as pms
from scipy.spatial.transform import Rotation

PM = pm.PointMatcher
DP = PM.DataPoints
Parameters = pms.Parametrizable.Parameters
output_file_path = 'libraries/libpointmatcher/evaluations/demo/'

async def start_ws(port: int, path: str, output: str, seed: int, number_of_random_transforms: int):
    async def run_eval(ws: WebSocketServerProtocol):
        # For each config in base64, we create a new file, use it for the eval then delete it
        async for config_base64 in ws:
            decoded_data = base64.b64decode(config_base64)
            full_output_file_path = output_file_path + 'config_' + str(uuid.uuid4)

            with open(full_output_file_path, 'wb') as f:
                f.write(decoded_data)

            scores = main(full_output_file_path, path, output, seed, number_of_random_transforms, send_via_websocket=True)
            await ws.send(json.dumps(scores))

            os.remove(output_file_path)

    async with serve(run_eval, "0.0.0.0", port):
        await asyncio.Future()


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


def main(config_file: str, path: str, output: str, seed: int, number_of_random_transforms: int, send_via_websocket: bool = False):
    if not os.path.exists(config_file):
        raise FileNotFoundError("The specified config file does not exist: {}".format(config_file))
    if not os.path.exists(path):
        raise FileNotFoundError("The specified point-cloud path does not exist: {}".format(path))
    if not os.path.exists(os.path.dirname(output)) and not send_via_websocket:
        raise FileNotFoundError(f"The output directory does not exist: {os.path.dirname(output)}")
    if os.path.isfile(output) and not send_via_websocket:
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

    if send_via_websocket:
        # Return the result
        return results_dict
    else: 
        # Writing to the output .json file
        with open(output, 'w', encoding='utf-8') as f:
            json.dump(results_dict, f, ensure_ascii=False, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Dummy script for libpointmatcher-server evaluation.'
                    'Use with absolute paths.'
                    'Your pipeline should accommodate output files with possible multiple data lines.')
    parser.add_argument('--ws', type=int, required=False,
                        help='[Optional] opens a Websocket on indicated port', metavar='PORT')
    parser.add_argument('--config', type=str, required=True,
                        help='Path to yaml configuration file')
    parser.add_argument('--path', type=str, required=True,
                        help='Path to a folder containing evaluation point clouds.'
                             'The clouds are organized into three folders: easy, medium and hard')
    parser.add_argument('--output', type=str, required=True,
                        help='Output path with score values')
    parser.add_argument('--seed', type=int, required=True,
                        help='Seed value')
    parser.add_argument('--iters', type=int,
                        help='Number of random transformations every point clouds is evaluated on.', default=10)

    args = parser.parse_args()
    try:
        if args.ws:
            asyncio.run(start_ws(args.ws, args.path, args.output, args.seed, args.iters))
        else:
            main(args.config, args.path, args.output, args.seed, args.iters)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    sys.exit(0)
