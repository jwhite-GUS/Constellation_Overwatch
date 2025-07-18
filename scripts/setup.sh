#!/bin/bash

# Galaxy Unmanned Systems - Environment Setup Script
# This script sets up the development environment

set -e

echo "🌟 Galaxy Unmanned Systems - ROS 2 Environment Setup"
echo "=================================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker Desktop and try again."
    exit 1
fi

echo "✅ Docker is running"

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null; then
    echo "❌ docker-compose is not installed. Please install Docker Desktop."
    exit 1
fi

echo "✅ docker-compose is available"

# Build the Docker images
echo "🔨 Building Docker images..."
docker-compose build

echo "🚀 Starting ROS 2 development environment..."
docker-compose up -d

echo "✅ Environment setup complete!"
echo ""
echo "💡 Next steps:"
echo "1. Open VS Code in this directory"
echo "2. Install the recommended VS Code extensions"
echo "3. Use 'docker-compose exec ros-dev bash' to enter the container"
echo "4. Run './scripts/build.sh' to build the workspace"
echo ""
echo "🎯 Happy coding!"
