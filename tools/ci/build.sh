#!/bin/bash
. "$( cd "$( dirname "$0" )" && pwd )/common.sh"

mkdir build
cd build
cmake ..

# Wraps the compilation with the Build Wrapper to generate configuration (used
# later by the SonarQube Scanner) into the "build" folder
build-wrapper-linux-x86-64 --out-dir build make

make install
