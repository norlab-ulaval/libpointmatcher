#!/bin/bash

PYTHON3_VERSION=$(python3 -c 'import sys; version=sys.version_info; print(f"{version.major}.{version.minor}")')

export PYTHON3_VERSION
