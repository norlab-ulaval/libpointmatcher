#!/usr/bin/env python3

import argparse
import os
import csv
import random
import sys


def main(config: str, point_cloud: str, output: str, seed: int):
    if not os.path.exists(config):
        raise FileNotFoundError("The specified config file does not exist: {}".format(config))
    if not os.path.exists(point_cloud):
        raise FileNotFoundError("The specified point-cloud file does not exist: {}".format(point_cloud))
    if not os.path.exists(os.path.dirname(output)):
        raise FileNotFoundError(f"Directory does not exist: {os.path.dirname(output)}")
    if os.path.isfile(output):
        raise FileExistsError(f"File already exists: {output}")

    # Create random scores
    easy_score = random.uniform(0, 1)
    medium_score = random.uniform(0, 1)
    hard_score = random.uniform(0, 1)

    # Calculate average score
    average_score = (easy_score + medium_score + hard_score) / 3

    # Fields (column names) and rows (values)
    fields = ['evaluation_name', 'easy', 'medium', 'hard', 'average']
    rows = [['cloud-to-cloud', easy_score, medium_score, hard_score, average_score]]

    # Writing to the csv file
    with open(output, mode='w', newline='') as csvfile:
        # creating a csv writer object
        csvwriter = csv.writer(csvfile)

        # writing the fields
        csvwriter.writerow(fields)

        # writing the data rows
        csvwriter.writerows(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Dummy script for libpointmatcher-server evaluation.'
                    'Use with absolute paths.'
                    'Your pipeline should accommodate output files with possible multiple data lines.')
    parser.add_argument('--config', type=str, required=True,
                        help='path to yaml configuration file')
    parser.add_argument('--point-cloud', type=str, required=True,
                        help='path to a point cloud')
    parser.add_argument('--output', type=str, required=True,
                        help='output path with score values')
    parser.add_argument('--seed', type=int, required=True,
                        help='seed value')

    args = parser.parse_args()
    try:
        main(args.config, args.point_cloud, args.output, args.seed)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    sys.exit(0)
