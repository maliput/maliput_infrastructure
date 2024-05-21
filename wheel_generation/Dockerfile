FROM quay.io/pypa/manylinux2014_x86_64

##########################################################################
# General setup
##########################################################################
RUN echo "export PATH=/opt/python/cp38-cp38/bin:\$PATH" >> /root/.bashrc
RUN export PATH=/opt/python/cp38-cp38/bin:$PATH

##########################################################################
# Install prerequisites.
##########################################################################

# colcon.
RUN /opt/python/cp38-cp38/bin/pip install colcon-common-extensions
RUN /opt/python/cp38-cp38/bin/pip install python-dev-tools

##########################################################################
# Install CMake dependencies
##########################################################################

# yaml-cpp
WORKDIR /source_builds/
RUN git clone -b yaml-cpp-0.7.0 https://github.com/jbeder/yaml-cpp.git yaml-cpp
WORKDIR /source_builds/yaml-cpp/build
RUN cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && make install

# tinyxml2
WORKDIR /source_builds/
RUN git clone -b 7.1.0 https://github.com/leethomason/tinyxml2.git tinyxml2
WORKDIR /source_builds/tinyxml2/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && make install

# pybind11
# See https://pybind11.readthedocs.io/en/stable/installing.html#include-with-pypi
# pybind11 (global) for pybind11 available directly in your environment root, e.g. /usr/local
# This brings a different version as the one used in ROS Foxy (2.4.3) however the global version isn't provided
# in the latter.
RUN /opt/python/cp38-cp38/bin/pip install "pybind11[global]==2.11.1"

##########################################################################
# Install Colcon packages
##########################################################################

WORKDIR /colcon_builds/src/

# ament_cmake
RUN git clone -b foxy https://github.com/ament/ament_cmake.git

# ament_package
RUN git clone -b foxy https://github.com/ament/ament_package.git

# eigen3_cmake_module
RUN yum install eigen3 -y
RUN git clone -b foxy https://github.com/ros2/eigen3_cmake_module.git

# Build colcon packages
WORKDIR /colcon_builds/
RUN PATH=/opt/python/cp38-cp38/bin:$PATH /opt/python/cp38-cp38/bin/colcon build --cmake-args " -DBUILD_TESTING=Off"
RUN source /colcon_builds/install/setup.bash
RUN echo "source /colcon_builds/install/setup.bash" >> /root/.bashrc

##########################################################################
# Install vcstool
##########################################################################
RUN /opt/python/cp38-cp38/bin/pip install vcstool

##########################################################################
# Final config
##########################################################################

RUN mkdir -p /ws/src
WORKDIR /ws

CMD ["/bin/bash"]
