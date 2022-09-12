# our local base image
FROM ubuntu:20.04

LABEL description="Container for use with Visual Studio" 
LABEL author="leeh8911@gmail.com"

USER root
ARG NUM_CORES=8

ENV DEBIAN_FRONTEND=noninteractive

## Install packages
RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y

# Basics
RUN apt-get install -qq -y ca-certificates
RUN apt-get install -qq -y libgoogle-glog-dev
RUN apt-get install -qq -y libgtest-dev
RUN apt-get install -qq -y automake
RUN apt-get install -qq -y wget
RUN apt-get install -qq -y curl
RUN apt-get install -qq -y unzip
RUN apt-get install -qq -y autoconf
RUN apt-get install -qq -y libtool
RUN apt-get install -qq -y g++ 
RUN apt-get install -qq -y cmake
RUN apt-get install -qq -y git
RUN apt-get install -qq -y clang-format-10
RUN apt-get install -qq -y clang-tidy
# BLAS & LAPACK
RUN apt-get install -qq -y libatlas-base-dev
# Eigen3
RUN apt-get install -qq -y libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
RUN apt-get install -qq -y libsuitesparse-dev
# Sphinx (for documentation)
RUN apt-get install -qq -y sphinx-doc sphinx-common

RUN apt-get install -qq -y clang clang-tidy clang-format
RUN apt-get install -qq -y cppcheck

WORKDIR /tmp
RUN git clone --depth=1 https://ceres-solver.googlesource.com/ceres-solver
WORKDIR /tmp/ceres-solver
RUN mkdir build
WORKDIR /tmp/ceres-solver/build
RUN cmake ..
RUN make -j ${NUM_CORES}
RUN make test
RUN make install

RUN cd /usr/src/googletest && \
    cmake . && \
    cmake --build . --target install

# Pangolin
RUN apt-get install -qq -y libtiff5-dev libjpeg8-dev zlib1g-dev \
libfreetype6-dev liblcms2-dev libwebp-dev libharfbuzz-dev libfribidi-dev \
tcl8.6-dev tk8.6-dev python-tk
# openGL
RUN apt-get install -qq -y freeglut3-dev libglu1-mesa-dev mesa-common-dev
RUN apt-get install -qq -y libglew-dev

WORKDIR /tmp
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /tmp/Pangolin
RUN chmod 744 ./scripts/install_prerequisites.sh
RUN mkdir build
WORKDIR /tmp/Pangolin/build
RUN cmake ..
RUN make -j ${NUM_CORES}
RUN make install
RUN ctest

# sophus
RUN apt-get install -qq -y libfmt-dev
WORKDIR /tmp
RUN git clone https://github.com/strasdat/Sophus.git
WORKDIR /tmp/Sophus
RUN mkdir build
WORKDIR /tmp/Sophus/build
RUN cmake ..
RUN make -j ${NUM_CORES}
RUN make install