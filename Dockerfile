FROM python:3.9-buster as builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y --no-install-recommends libboost-all-dev \
        git \
        wget \
        g++ \
        gcc \
        make \
        libeigen3-dev \
        ninja-build \
        catch \
        libomp-dev \
        gosu \
        less

ARG CMAKE_VERSION=3.22.1

# Install CMake
RUN wget -q -O ./cmake.sh https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh && \
    mkdir /opt/cmake && \
    sh ./cmake.sh --prefix=/opt/cmake --skip-license && \
    ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake && \
    rm ./cmake.sh

ARG LIB_DIR=/home/build/Libraries

WORKDIR ${LIB_DIR}

ARG LIBNABO_VERSION=1.0.7

RUN git clone -b ${LIBNABO_VERSION} --single-branch https://github.com/ethz-asl/libnabo.git && \
    cd libnabo && \
    cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DUSE_OPEN_MP:BOOL=ON -S . -B ./build && \
    cmake --build ./build --target install --parallel $(nproc) && \
    cd .. && \
    rm -r ./libnabo


# Use only 2.5.0
RUN git clone -b v2.5.0 --single-branch https://github.com/pybind/pybind11.git && \
    cd pybind11 && \
    cmake -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYBIND11_TEST:BOOL=OFF -S . -B ./build && \
    cmake --build ./build --target install --parallel $(nproc) && \
    cd .. && \
    rm -r ./pybind11

COPY . .

# RUN git clone -b feature/python-package --single-branch https://github.com/phygitalism/libpointmatcher.git && \
#     cd libpointmatcher && \
#     cmake -DBUILD_PYTHON_MODULE:BOOL=ON -DPYTHON_INSTALL_TARGET=$(pwd)/python/pypointmatcher_native -DUSE_OPEN_MP:BOOL=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -S . -B ./build && \
#     cmake --build ./build --target install --parallel $(nproc)
    # cd .. && \
    # rm -r ./libpointmatcher

RUN cmake -DBUILD_PYTHON_MODULE:BOOL=ON -DPYTHON_INSTALL_TARGET=$(pwd)/python/pypointmatcher_native -DUSE_OPEN_MP:BOOL=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -S . -B ./build && \
    cmake --build ./build --target install --parallel $(nproc)

ARG WHEEL_DIR=/home/build-python

COPY ./python/setup.py ./libpointmatcher/python/setup.py

RUN cd ./libpointmatcher/python && \
    python ./setup.py bdist_wheel -d ${WHEEL_DIR}

FROM python:3.9-slim

ARG WHEEL_DIR=/home/build-python

COPY --from=builder ${WHEEL_DIR} ${WHEEL_DIR}

RUN pip install ${WHEEL_DIR}/*.whl

