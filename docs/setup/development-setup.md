# Development Setup Guide

This guide will help you set up the Galaxy Unmanned Systems ROS 2 development environment on your local machine.

## Prerequisites Verification

Before starting, verify that you have the required software installed:

### Docker Desktop
1. Open a terminal/command prompt
2. Run: `docker --version`
3. You should see output like: `Docker version 24.0.x, build xxxxx`

### Git
1. Run: `git --version`
2. You should see output like: `git version 2.x.x`

### VS Code
1. Open VS Code
2. Check that it opens without errors

## Step-by-Step Setup

### 1. Clone the Repository
```bash
git clone <your-github-repo-url>
cd ROS
```

### 2. Open in VS Code
```bash
code gus-ros-workspace.code-workspace
```

### 3. Install Recommended Extensions
VS Code should prompt you to install the recommended extensions. Click "Install All" or install them manually:

- ROS
- Docker
- C/C++ Extension Pack
- Python
- CMake Tools
- YAML

### 4. Start the Development Environment

#### Option A: Using the Setup Script
```bash
# On Windows (PowerShell)
./scripts/setup.sh

# On macOS/Linux
chmod +x scripts/setup.sh
./scripts/setup.sh
```

#### Option B: Manual Setup
```bash
# Build the Docker images
docker-compose build

# Start the containers
docker-compose up -d

# Enter the development container
docker-compose exec ros-dev bash
```

### 5. Build the Workspace
Once inside the container:
```bash
# Make the build script executable
chmod +x scripts/build.sh

# Run the build script
./scripts/build.sh
```

## Verifying the Setup

### 1. Check ROS 2 Installation
```bash
ros2 --help
```

### 2. Check Available Packages
```bash
ros2 pkg list
```

### 3. Test Gazebo (if using simulation)
```bash
# In one terminal
ros2 launch gazebo_ros gazebo.launch.py

# In another terminal
ros2 run gazebo_ros spawn_entity.py -help
```

## Development Workflow

1. **Edit code** in VS Code on your host machine
2. **Build** inside the Docker container: `./scripts/build.sh`
3. **Run** ROS 2 nodes inside the container
4. **Test** your changes
5. **Commit** changes using git on your host machine

## Troubleshooting

### Common Issues

**Docker Desktop won't start:**
- Ensure virtualization is enabled in BIOS
- On Windows, ensure WSL2 is installed
- Restart your computer

**Permission errors:**
- On Windows, run PowerShell as Administrator
- On macOS/Linux, check file permissions

**ROS 2 commands not found:**
- Make sure you're inside the Docker container
- Source the ROS 2 setup: `source /opt/ros/humble/setup.bash`

**Build failures:**
- Check that all dependencies are installed
- Try cleaning the workspace: `rm -rf build install log`

Need help? Contact the development team or check the main README.md for more information.
