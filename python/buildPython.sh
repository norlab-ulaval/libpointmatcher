#!/bin/bash
pip uninstall pypointmatcher -y && \
cd .. && \
rm -rf build_python && \
mkdir build_python && \
cd build_python && \
cmake -D BUILD_PYTHON_MODULE=ON .. # -Dpybind11_DIR=/Volumes/CaseSensitive/libpointmatcher/.venv/lib/python3.10/site-packages/pybind11/share/cmake/pybind11 .. && \
make -j12 && \
sudo make install && \
cd ../python && \
rm -rf dist && \
python -m build --wheel --no-isolation --outdir ./dist && \
pip install dist/* && \
rm -rf dist build *.egg-info && \
python -c "from pypointmatcher import *"
