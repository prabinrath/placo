# Building Placo for ROS Humble Pinocchio

This guide explains how to build and install placo to work with the ROS Humble version of pinocchio in the lerobot conda environment.

## Prerequisites

- ROS Humble installed with pinocchio (`ros-humble-pinocchio`)
- Conda environment for lerobot
- Ubuntu 22.04 (Jammy)

## Install Dependencies

```bash
# Install ROS eiquadprog (required by placo)
sudo apt-get install -y ros-humble-eiquadprog

# Install doxygen and doxystub for stub generation
sudo apt-get install -y doxygen
pip install doxystub
```

## Source Code Fix

The ROS Humble hpp-fcl uses `hpp::fcl` namespace, but placo uses the newer `coal` namespace. You need to patch `src/placo/model/robot_wrapper.cpp`:

1. Add this include after the other includes:
```cpp
#include "hpp/fcl/collision_data.h"
```

2. Replace `coal::CollisionResult` with `hpp::fcl::CollisionResult` (around line 462)

3. Replace `coal::DistanceResult` with `hpp::fcl::DistanceResult` (around line 502)

## Build

```bash
cd <path-to-placo>

# Create build directory
mkdir -p build && cd build

# Source ROS Humble
source /opt/ros/humble/setup.bash

# Configure with CMake (pointing to ROS pinocchio)
cmake .. \
    -DCMAKE_PREFIX_PATH=/opt/ros/humble \
    -DPYTHON_SITELIB=lib/python3.10/site-packages \
    -DCMAKE_INSTALL_PREFIX=<path-to-placo>/install

# Build
make -j$(nproc)
```

## Install to Lerobot Environment

Copy the built files to the lerobot conda environment:

```bash
# Get the site-packages path for your conda environment
LEROBOT_SITEPACKAGES=$(conda run -n lerobot python -c "import site; print(site.getsitepackages()[0])")

# Copy placo module and dependencies
cp <path-to-placo>/build/lib/python3.10/site-packages/placo.so $LEROBOT_SITEPACKAGES/
cp <path-to-placo>/build/lib/python3.10/site-packages/placo.pyi $LEROBOT_SITEPACKAGES/
cp -r <path-to-placo>/build/lib/python3.10/site-packages/placo_utils $LEROBOT_SITEPACKAGES/
cp <path-to-placo>/build/liblibplaco.so $LEROBOT_SITEPACKAGES/
```

## Numpy Compatibility

The ROS pinocchio was built with numpy 1.x. You need to downgrade numpy in the lerobot environment:

```bash
conda activate lerobot
pip install "numpy<2"
pip install "opencv-python<4.11" "opencv-python-headless<4.11"
```

## Usage

When running lerobot with placo, you must source ROS Humble first:

```bash
# Manual setup
source /opt/ros/humble/setup.bash
conda activate lerobot
```

## Verify Installation

```bash
source /opt/ros/humble/setup.bash
conda activate lerobot

python -c "
import pinocchio
print('pinocchio version:', pinocchio.__version__)
import placo
print('placo: ok')
from lerobot.model.kinematics import RobotKinematics
print('RobotKinematics: ok')
"
```

Expected output:
```
pinocchio version: 3.8.0
placo: ok
RobotKinematics: ok
```

## Notes

- The ROS pinocchio is located at `/opt/ros/humble/lib/python3.10/site-packages/pinocchio/`
- Placo must be built against the same pinocchio version used at runtime
- Always source ROS before activating conda to ensure correct library paths
