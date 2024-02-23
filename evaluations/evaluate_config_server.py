#!/usr/bin/env python3

import argparse
import asyncio
import json
import os
import csv
import random
import sys
from websockets import serve, WebSocketServerProtocol


def evaluate() -> dict[str, float]:
    # Create random scores
    easy_score = random.uniform(0, 1)
    medium_score = random.uniform(0, 1)
    hard_score = random.uniform(0, 1)

    # Calculate average score
    average_score = (easy_score + medium_score + hard_score) / 3

    scores = {
        'easy': easy_score,
        'medium': medium_score,
        'hard': hard_score,
        'average': average_score
    }

    return scores


async def start_ws(port: int):
    async def run_eval(ws: WebSocketServerProtocol):
        async for _ in ws:
            scores = evaluate()
            await ws.send(json.dumps(scores))

    async with serve(run_eval, "0.0.0.0", port):
        await asyncio.Future()


def main(config: str, point_cloud: str, output: str, seed: int):
    if not os.path.exists(config):
        raise FileNotFoundError("The specified config file does not exist: {}".format(config))
    if not os.path.exists(point_cloud):
        raise FileNotFoundError("The specified point-cloud file does not exist: {}".format(point_cloud))
    if not os.path.exists(os.path.dirname(output)):
        raise FileNotFoundError(f"Directory does not exist: {os.path.dirname(output)}")
    if os.path.isfile(output):
        raise FileExistsError(f"File already exists: {output}")

    scores = evaluate()

    fields = ['evaluation_name']
    first_row = ['cloud-to-cloud']

    for field_name in scores.keys():
        fields.append(field_name)
        first_row.append(scores[field_name])

    rows = [first_row]

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
    parser.add_argument('--ws', type=int, required=False,
                        help='[Optional] opens a Websocket on indicated port', metavar='PORT')
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
        if args.ws:
            asyncio.run(start_ws(args.ws))
        else:
            main(args.config, args.point_cloud, args.output, args.seed)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

    sys.exit(0)
