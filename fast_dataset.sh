#!/bin/bash
# filepath: /mnt/D/code/python/vio/fast_dataset.sh

# Build the project
./build.sh

# Source the setup
source install/setup.sh

# Create dataset directory in project folder
mkdir -p dataset

# Run fast dataset collection with parameters
ros2 launch vio fast_dataset_collection.launch.py \
    world:=forest1.sdf \
    pattern:=aggressive \
    max_speed:=20.0 \
    dataset_path:=$(pwd)/dataset \
    record_rate:=10.0