# Contributing to Constellation Overwatch SDK

<!-- COPILOT: Contributing guidelines for professional government/community project -->
<!-- COPILOT: Maintain professional tone and comprehensive process documentation -->

Thank you for your interest in contributing to the Constellation Overwatch SDK! This document provides guidelines for contributing to our open-source, government-owned autonomous systems integration platform.

## Mission Statement

Constellation Overwatch is a community-driven effort to create a modular open systems architecture (MOSA) for autonomous systems integration. Our goal is to provide open, non-proprietary tools that enable rapid, secure integration of heterogeneous unmanned systems and payloads, eliminating proprietary licensing barriers and fostering innovation.

## Community Values

- **Open Source First**: Everything we build is open source and freely available
- **Government Ownership**: Ensuring government and public access to critical technologies
- **Community Driven**: Welcoming contributions from all stakeholders
- **Security Focused**: Maintaining the highest security standards
- **Interoperability**: Enabling seamless integration across diverse systems

## Development Process

### 1. Setting Up Your Development Environment

1. **Fork the repository** (if working with external contributors)
2. **Clone your fork** or the main repository
3. **Follow the setup guide** in `docs/setup/development-setup.md`
4. **Verify your setup** by building the workspace

### 2. Making Changes

1. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** following our coding standards
3. **Test your changes** thoroughly
4. **Commit your changes** with clear, descriptive messages

### 3. Submitting Your Changes

1. **Push your branch** to your fork (or the main repository if you have access)
2. **Create a Pull Request** with:
   - Clear description of what you've changed
   - Why the changes are needed
   - Any testing you've performed
   - Screenshots or videos if applicable

## Coding Standards

### General Guidelines

- **Follow ROS 2 conventions** for naming and structure
- **Write clear, self-documenting code**
- **Include comments** for complex logic
- **Use consistent indentation** (4 spaces for Python, 2 spaces for C++)

### Python Code Standards

- Follow **PEP 8** style guidelines
- Use **type hints** where appropriate
- Include **docstrings** for all functions and classes
- Use **meaningful variable names**

Example:
```python
def calculate_drone_position(
    current_position: geometry_msgs.msg.Point,
    velocity: geometry_msgs.msg.Vector3,
    time_delta: float
) -> geometry_msgs.msg.Point:
    """
    Calculate the new drone position based on current position and velocity.
    
    Args:
        current_position: Current 3D position of the drone
        velocity: Current velocity vector
        time_delta: Time elapsed since last update
        
    Returns:
        New calculated position
    """
    # Implementation here
    pass
```

### C++ Code Standards

- Follow **Google C++ Style Guide**
- Use **modern C++17** features
- Include **header guards** in all header files
- Use **smart pointers** instead of raw pointers

Example:
```cpp
#ifndef DRONE_CONTROLLER_HPP_
#define DRONE_CONTROLLER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gus_drone_control {

class DroneController : public rclcpp::Node {
public:
    explicit DroneController(const std::string& node_name);
    
    /**
     * @brief Send velocity command to drone
     * @param velocity Target velocity
     */
    void sendVelocityCommand(const geometry_msgs::msg::Twist& velocity);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};

}  // namespace gus_drone_control

#endif  // DRONE_CONTROLLER_HPP_
```

### ROS 2 Package Structure

Each package should follow this structure:
```
package_name/
├── package.xml
├── CMakeLists.txt (for C++) or setup.py (for Python)
├── launch/
│   └── package_launch_files.launch.py
├── config/
│   └── parameters.yaml
├── src/
│   └── source_files
├── include/
│   └── header_files (C++ only)
├── test/
│   └── test_files
└── README.md
```

### Launch Files

- Use **Python launch files** (.launch.py) for ROS 2
- Include **clear parameter descriptions**
- Use **yaml configuration files** for parameters

### Configuration Files

- Use **YAML format** for configuration
- Include **comments** explaining each parameter
- Group related parameters logically

## Testing

### Unit Tests

- Write **unit tests** for all new functionality
- Use **pytest** for Python tests
- Use **gtest** for C++ tests
- Aim for **>80% code coverage**

### Integration Tests

- Test **ROS 2 node interactions**
- Test **simulation environments**
- Verify **cross-platform compatibility**

### Testing Commands

```bash
# Run all tests
./scripts/test.sh

# Run specific test
colcon test --packages-select your_package_name

# View test results
colcon test-result --verbose
```

## Documentation

### Code Documentation

- Include **docstrings** for all public functions
- Use **inline comments** for complex logic
- Document **ROS 2 interfaces** (topics, services, actions)

### User Documentation

- Update **README.md** if adding new features
- Add **tutorial files** for new functionality
- Include **examples** in documentation

### API Documentation

- Document **public APIs** thoroughly
- Include **usage examples**
- Specify **input/output formats**

## Commit Message Guidelines

Use clear, descriptive commit messages:

```
feat: Add drone position controller

- Implement PID controller for position control
- Add configuration parameters for tuning
- Include unit tests for controller logic
- Update documentation with usage examples

Resolves: #123
```

### Commit Types

- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation changes
- `style:` Code style changes (formatting, etc.)
- `refactor:` Code refactoring
- `test:` Adding or updating tests
- `chore:` Maintenance tasks

## Review Process

### Pull Request Requirements

- [ ] Code follows style guidelines
- [ ] Tests pass locally
- [ ] Documentation updated
- [ ] No merge conflicts
- [ ] Descriptive PR title and description

### Review Checklist

Reviewers should check:
- [ ] Code quality and style
- [ ] Test coverage
- [ ] Documentation completeness
- [ ] Performance considerations
- [ ] Security implications
- [ ] Cross-platform compatibility

## Getting Help

### Communication Channels

- **GitHub Issues**: For bug reports and feature requests
- **GitHub Discussions**: For general questions and discussions
- **Team Slack/Discord**: For real-time communication (if applicable)

### Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Docker Documentation](https://docs.docker.com/)
- [VS Code ROS Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.ros)

## License

By contributing to this project, you agree that your contributions will be licensed under the same license as the project (Apache 2.0).

## Questions?

If you have any questions about contributing, please:
1. Check the existing documentation
2. Search through existing issues
3. Ask in the appropriate communication channel
4. Create a new issue if needed

Thank you for contributing to Galaxy Unmanned Systems! 🚁✨
