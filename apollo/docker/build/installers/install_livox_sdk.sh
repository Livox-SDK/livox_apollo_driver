#!/usr/bin/env bash

###############################################################################
# Copyright 2020 Livox. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -e

SDK_VERSION_MAJOR=2
SDK_VERSION_MINOR=0
SDK_VERSION_PATCH=0

cd "$(dirname "${BASH_SOURCE[0]}")"
ARCH=$(uname -m)
if [ "$ARCH" == "aarch64" ]; then
  BUILD=$1
  shift
fi

if [ "$BUILD" == "build" ] || [ "$ARCH" == "x86_64" ]; then
  sudo apt install -y libapr1-dev
  if [ ! -f /usr/lib/x86_64-linux-gnu/libboost_system.so  ]; then
  ln -s /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1 /usr/lib/x86_64-linux-gnu/libboost_system.so
  fi
  wget https://github.com/Livox-SDK/Livox-SDK/archive/v${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}.tar.gz

  tar xzvf v${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}.tar.gz

  pushd Livox-SDK-${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}/
  echo "project(livox_sdk)" >> temp
  echo "set(CMAKE_CXX_STANDARD 11)" >> temp
  cat ./sdk_core/CMakeLists.txt|sed 's/_static/_shared/'|sed 's/STATIC/SHARED/' >> temp
  mv temp ./sdk_core/CMakeLists.txt
  cd build
  cmake ../sdk_core
  make -j2
  make install
  popd
  rm -rf Livox-SDK-${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}
  rm v${SDK_VERSION_MAJOR}.${SDK_VERSION_MINOR}.${SDK_VERSION_PATCH}.tar.gz*
fi

