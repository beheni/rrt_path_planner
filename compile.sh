#!/bin/bash -x
install_prefix=".."
mkdir -p ./cmake-build-release
(
  pushd ./cmake-build-release > /dev/null || exit 1
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${install_prefix}" .. || exit 1
  cmake --build . || exit 1
  cmake --install . || exit 1
  popd || exit 1
)
