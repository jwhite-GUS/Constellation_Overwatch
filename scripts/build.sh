#!/bin/bash

# Galaxy Unmanned Systems - ROS 2 Build Script
# This script builds all ROS 2 packages in the workspace

set -e

echo "🚀 Starting ROS 2 workspace build..."

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Change to workspace directory
cd /workspace

# Install dependencies
echo "📦 Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "🔨 Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "🔧 Sourcing workspace..."
source install/setup.bash

echo "✅ Build completed successfully!"
echo "💡 Remember to source the workspace: source install/setup.bash"
