#!/bin/bash
set -e

pushd /apollo/modules/localization

# 动态库更新到主仓
  mkdir -p dist/aarch64/bin
  cp /apollo/bazel-out/aarch64-opt/bin/modules/localization/src/dead_reckoning/libDR_component.so dist/aarch64/bin/
  cp /apollo/bazel-out/aarch64-opt/bin/modules/localization/src/TCMSF/libTCMSF_component.so dist/aarch64/bin/
  mkdir -p dist/x86_64/bin
  cp /apollo/bazel-out/k8-opt/bin/modules/localization/src/dead_reckoning/libDR_component.so dist/x86_64/bin/
  cp /apollo/bazel-out/k8-opt/bin/modules/localization/src/TCMSF/libTCMSF_component.so dist/x86_64/bin/

  mkdir -p dist/aarch64/libs
  cp src/TCMSF/third_party/wgs84_to_mars/lib/arm/libkcoords_plugin.so dist/aarch64/libs/
  mkdir -p dist/x86_64/libs
  cp src/TCMSF/third_party/wgs84_to_mars/lib/x86/libkcoords_plugin.so dist/x86_64/libs/

  mkdir -p dist/aarch64/conf
  cp src/TCMSF/conf/* dist/aarch64/conf/
  cp src/dead_reckoning/conf/* dist/aarch64/conf/
  mkdir -p dist/x86_64/conf
  cp src/TCMSF/conf/* dist/x86_64/conf/
  cp src/dead_reckoning/conf/* dist/x86_64/conf/

  mkdir -p dist/aarch64/dag
  cp src/TCMSF/dag/* dist/aarch64/dag/
  cp src/dead_reckoning/dag/* dist/aarch64/dag/
  mkdir -p dist/x86_64/dag
  cp src/TCMSF/dag/* dist/x86_64/dag/
  cp src/dead_reckoning/dag/* dist/x86_64/dag/

  dvc add dist

  dvc push

popd
