# Galaxy Unmanned Systems - ROS Drone Simulation Platform

A cross-platform ROS 2 development environment for drone simulation and control, designed to work seamlessly on Windows and macOS using Docker containerization.

## 🎯 Project Overview

This repository contains the infrastructure and code for developing drone simulations that can interface with various physics engines including Gazebo, Unity, and Unreal Engine. The project is built on ROS 2 and uses Docker to ensure consistent behavior across different operating systems.

## 🏗️ Architecture

- **ROS 2 Humble**: The core robotics framework with cross-platform support
- **Docker**: Containerized environment for consistent deployment
- **Physics Engines**: Integration with Gazebo, Unity, and Unreal Engine
- **Cross-Platform**: Native support for Windows 10/11 and macOS

## 📋 Prerequisites

### Required Software (Install on Host Machine)

#### For All Platforms:
1. **Visual Studio Code** (Latest version)
   - Download from: https://code.visualstudio.com/

2. **Docker Desktop** (Latest version)
   - Windows: https://docs.docker.com/desktop/windows/install/
   - macOS: https://docs.docker.com/desktop/mac/install/
   - **Important**: Ensure Docker Desktop is running before starting development

3. **Git** (Latest version)
   - Windows: https://git-scm.com/download/win
   - macOS: `brew install git` or download from https://git-scm.com/download/mac

#### Platform-Specific Requirements:

**Windows:**
- Windows 10/11 (64-bit)
- WSL2 (Windows Subsystem for Linux) - Required for Docker Desktop
- PowerShell 5.1 or later

**macOS:**
- macOS 10.15 (Catalina) or later
- Xcode Command Line Tools: `xcode-select --install`

### Recommended VS Code Extensions

Install these extensions for the best development experience:

```bash
# Essential Extensions
code --install-extension ms-vscode.ros
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-vscode.cpptools-extension-pack
code --install-extension ms-python.python
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.yaml
```

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone <repository-url>
cd ROS
```

### 2. Verify Docker Installation

```bash
docker --version
docker-compose --version
```

### 3. Build the ROS Environment

```bash
# Build the Docker containers
docker-compose build

# Start the development environment
docker-compose up -d
```

### 4. Access the Development Environment

```bash
# Enter the ROS container
docker-compose exec ros-dev bash

# Verify ROS 2 installation
ros2 --help
```

## 📁 Project Structure

```
ROS/
├── README.md                 # This file
├── docker/                   # Docker configuration files
│   ├── Dockerfile.ros        # Main ROS 2 container
│   ├── Dockerfile.gazebo     # Gazebo simulation container
│   └── docker-compose.yml    # Multi-container orchestration
├── src/                      # ROS 2 source code
│   ├── drone_control/        # Drone control packages
│   ├── simulation/           # Simulation packages
│   └── interfaces/           # Custom message definitions
├── config/                   # Configuration files
│   ├── params/               # ROS parameters
│   └── launch/               # Launch files
├── scripts/                  # Utility scripts
│   ├── setup.sh              # Environment setup script
│   └── build.sh              # Build script
├── docs/                     # Documentation
│   ├── setup/                # Setup guides
│   ├── tutorials/            # Tutorials
│   └── api/                  # API documentation
└── tests/                    # Test files
    ├── unit/                 # Unit tests
    └── integration/          # Integration tests
```

## 🔧 Development Workflow

### Setting Up Your Development Environment

1. **Open VS Code in the project directory**
   ```bash
   code .
   ```

2. **Open the integrated terminal** (Ctrl+` or Cmd+`)

3. **Start the Docker environment**
   ```bash
   docker-compose up -d
   ```

4. **Attach VS Code to the container** (using the Docker extension)

### Building and Running

```bash
# Build all packages
./scripts/build.sh

# Run a specific simulation
ros2 launch simulation gazebo_drone.launch.py

# Run tests
./scripts/test.sh
```

## 🎮 Simulation Environments

### Gazebo Classic
- **Purpose**: Default physics simulation
- **Launch**: `ros2 launch simulation gazebo_world.launch.py`
- **Features**: Built-in physics, sensor models, world environments

### Unity Integration
- **Purpose**: High-fidelity visual simulation
- **Setup**: See `docs/setup/unity-integration.md`
- **Communication**: ROS-TCP-Connector

### Unreal Engine Integration
- **Purpose**: Photorealistic simulation
- **Setup**: See `docs/setup/unreal-integration.md`
- **Communication**: ROS 2 bridge

## 🐛 Troubleshooting

### Common Issues

**Docker Desktop not starting:**
- Ensure virtualization is enabled in BIOS
- On Windows, ensure WSL2 is properly installed
- Restart Docker Desktop service

**ROS 2 commands not found:**
- Ensure you're inside the Docker container
- Source the ROS 2 setup: `source /opt/ros/humble/setup.bash`

**Build failures:**
- Check Docker container logs: `docker-compose logs ros-dev`
- Ensure all dependencies are installed in the container

**Performance issues:**
- Allocate more resources to Docker Desktop
- Close unnecessary applications

### Getting Help

1. Check the `docs/` directory for detailed guides
2. Review Docker container logs for error messages
3. Contact the development team via [communication channel]

## 📚 Documentation

- [Development Setup Guide](docs/setup/development-setup.md)
- [Docker Environment Guide](docs/setup/docker-guide.md)
- [ROS 2 Package Development](docs/tutorials/ros2-packages.md)
- [Simulation Integration](docs/tutorials/simulation-setup.md)
- [API Documentation](docs/api/README.md)

## 🤝 Contributing

1. Create a feature branch from `main`
2. Make your changes following the coding standards
3. Test your changes thoroughly
4. Submit a pull request with a clear description

### Coding Standards

- Follow ROS 2 naming conventions
- Use consistent indentation (4 spaces)
- Include docstrings for all functions
- Write unit tests for new features

## 📄 License

[Add your license information here]

## 👥 Team

- **Project Lead**: [Name]
- **ROS Development**: [Name]
- **Simulation**: [Name]
- **Infrastructure**: [Name]

## 🔄 Version History

- **v0.1.0**: Initial project setup and Docker configuration
- **v0.2.0**: Basic ROS 2 package structure
- **v0.3.0**: Gazebo integration
- **v1.0.0**: First stable release

---

**Note**: This project is designed to work identically on Windows and macOS. If you encounter platform-specific issues, please document them and notify the team.
