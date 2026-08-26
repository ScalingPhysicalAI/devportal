#!/usr/bin/env bash
# Copies the MuJoCo WASM engine (from node_modules/@mujoco/mujoco, installed
# via package.json) and the converted robot model (model/mjcf/, produced by
# model/scripts/urdf_to_mjcf.py) into public/mujoco/ so the browser can fetch
# them as static assets at runtime. Re-run after upgrading @mujoco/mujoco or
# regenerating model/mjcf/.
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p public/mujoco/engine public/mujoco/scene
cp node_modules/@mujoco/mujoco/mujoco.js node_modules/@mujoco/mujoco/mujoco.wasm public/mujoco/engine/
rm -rf public/mujoco/scene/meshes
cp model/mjcf/humanoid.xml public/mujoco/scene/humanoid.xml
cp -r model/mjcf/meshes public/mujoco/scene/meshes

echo "synced $(du -sh public/mujoco | cut -f1) into public/mujoco/"
