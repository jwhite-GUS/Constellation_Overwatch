# Constellation Overwatch SDK

**Government-Owned, Community-Driven Autonomous Systems Integration Platform**

Constellation Overwatch is a modular open systems architecture (MOSA) autonomy and integration stack designed to support distributed command and control of autonomous systems, sensors, and payloads. This SDK provides a common, open-source development environment enabling rapid, secure integration of heterogeneous unmanned systems and payloads into a unified operational picture.

## 🎯 Project Overview

Constellation Overwatch differentiates itself through its completely open, non-proprietary licensing and ecosystem-driven development model. Leveraging existing government-owned solutions such as the Team Awareness Kit (TAK), this platform enables real-time data fusion, collaborative autonomy, multi-domain sensor integration, and intuitive common operating picture (COP) visualization—without restrictive per-seat licenses or proprietary integration barriers.

**Functionally analogous to**: Anduril's Lattice, Collins Aerospace's RapidEdge, Boeing's collaborative autonomy systems, and commercial industry Pixhawk standards, but with complete open-source accessibility.

## 🔑 Key Attributes

### **Open Architecture Integration**
- Rapid "plug-and-play" capability for any MOSA-compliant autonomous platform, payload, or sensor
- Standardized interfaces and communication protocols
- Hardware-agnostic design supporting diverse unmanned systems

### **Continuous ATO & Secure DevSecOps Environment**
- Government-owned sandbox for rapid, secure innovation and deployment
- Containerized development environment with security best practices
- Automated testing and continuous integration pipelines

### **Collaborative Autonomy & Distributed AI**
- Real-time tasking and mission autonomy across diverse unmanned platforms
- Multi-domain sensor fusion and data processing
- Distributed decision-making capabilities

### **Ecosystem-Driven Development**
- Community-contributed capabilities and plugins accelerate adoption
- Modular architecture enabling rapid capability integration
- Reduced development costs through shared resources

### **Common Operational Picture (COP) Integration**
- Seamless compatibility with existing government COP frameworks (e.g., TAK)
- Real-time data visualization and situational awareness
- Interoperable with existing military command and control systems

## 🎖️ Mission Impact

With new mandates for mass-proliferation of US-made commoditized sUAS, Constellation Overwatch allows government and industry partners to quickly integrate capabilities into operationally relevant scenarios, maximizing innovation and interoperability while eliminating proprietary licensing lock-in. More importantly, it gets UXS tools into the hands of warfighters directly by decentralizing the integration of useful robotics devices into common portable unit-level kits.

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

## 📁 SDK Structure

```
constellation-overwatch-sdk/
├── README.md                    # This file - SDK overview and documentation
├── LICENSE                      # Apache 2.0 + Constellation Overwatch terms
├── CONTRIBUTING.md              # Community contribution guidelines
├── CHANGELOG.md                 # Version history and updates
├── docker-compose.yml           # Multi-container development environment
├── constellation-overwatch.code-workspace  # VS Code workspace configuration
├── sdk/                         # Core SDK components
│   ├── core/                    # Core SDK functionality
│   │   ├── autonomy/            # Autonomous system interfaces
│   │   ├── communication/       # Inter-system communication protocols
│   │   ├── fusion/              # Sensor fusion and data integration
│   │   ├── security/            # Security and authentication modules
│   │   └── orchestration/       # Mission planning and execution
│   ├── interfaces/              # Standard interfaces and APIs
│   │   ├── mosa/                # MOSA-compliant interfaces
│   │   ├── tak/                 # Team Awareness Kit integration
│   │   ├── mavlink/             # MAVLink protocol support
│   │   └── common/              # Common interface definitions
│   ├── plugins/                 # Extensible plugin system
│   │   ├── sensors/             # Sensor integration plugins
│   │   ├── payloads/            # Payload management plugins
│   │   ├── platforms/           # Platform-specific plugins
│   │   └── custom/              # Custom user plugins
│   └── tools/                   # Development and deployment tools
│       ├── simulator/           # Simulation environment tools
│       ├── configurator/        # System configuration utilities
│       ├── monitor/             # System monitoring and diagnostics
│       └── deployer/            # Deployment and packaging tools
├── examples/                    # Example implementations
│   ├── basic-drone/             # Basic drone integration example
│   ├── sensor-fusion/           # Multi-sensor fusion example
│   ├── swarm-coordination/      # Swarm coordination example
│   └── mission-planning/        # Mission planning example
├── samples/                     # Sample code and templates
│   ├── plugins/                 # Plugin development samples
│   ├── integrations/            # Integration samples
│   └── configurations/         # Configuration templates
├── integrations/                # Third-party integrations
│   ├── gazebo/                  # Gazebo simulation integration
│   ├── unity/                   # Unity engine integration
│   ├── unreal/                  # Unreal Engine integration
│   └── hardware/                # Hardware platform integrations
├── docs/                        # Comprehensive documentation
│   ├── sdk-guide/               # SDK development guide
│   ├── api-reference/           # API documentation
│   ├── tutorials/               # Step-by-step tutorials
│   ├── integration-guides/      # Integration guides
│   └── security/                # Security documentation
├── tests/                       # Comprehensive test suite
│   ├── unit/                    # Unit tests
│   ├── integration/             # Integration tests
│   ├── performance/             # Performance tests
│   └── security/                # Security tests
├── scripts/                     # Utility scripts
│   ├── setup.sh                 # Linux/macOS setup script
│   ├── setup.ps1                # Windows PowerShell setup script
│   ├── build.sh                 # Build script
│   ├── test.sh                  # Test runner script
│   └── deploy.sh                # Deployment script
└── config/                      # Configuration files
    ├── development/             # Development environment configs
    ├── production/              # Production environment configs
    └── security/                # Security configurations
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
