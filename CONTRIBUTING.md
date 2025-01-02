# Contributing to Containerized Autonomous Agricultural Management System

## Table of Contents
- [Introduction](#introduction)
- [Development Environment Setup](#development-environment-setup)
- [Development Workflow](#development-workflow)
- [Contribution Guidelines](#contribution-guidelines)
- [Issue Guidelines](#issue-guidelines)

## Introduction

The Containerized Autonomous Agricultural Management System is a safety-critical platform integrating NVIDIA's DeepStream, TAO Toolkit, ROS 2, and Gazebo for autonomous agricultural operations. Due to the real-time and safety-critical nature of our system, we maintain strict development standards to ensure reliability and performance.

### Code of Conduct
All contributors must adhere to our Code of Conduct, emphasizing responsible development practices for autonomous systems. Safety and reliability are our top priorities.

### System Architecture Overview
Our system utilizes containerized microservices running on Jetson Orin hardware, with strict real-time processing requirements and safety considerations for autonomous operations.

## Development Environment Setup

### Hardware Requirements
- Jetson Orin AGX (32GB+ RAM)
- NVIDIA GPU with Ampere architecture
- Adequate storage for development containers (500GB+ NVMe recommended)

### Software Requirements
- Docker v23.0+
- NVIDIA Container Toolkit v1.13+
- CMake v3.25+
- Python v3.10+
- ROS 2 JAZZY

### Environment Configuration
1. Install NVIDIA Container Toolkit:
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

2. Configure Docker for GPU access:
```bash
sudo systemctl restart docker
```

3. Verify GPU access:
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## Development Workflow

### Branch Naming Convention
- Features: `feature/<component>/<feature-name>`
- Bug fixes: `fix/<component>/<bug-name>`
- Hotfixes: `hotfix/<component>/<issue-name>`
- Releases: `release/<version>`
- Safety features: `safety/<component>/<feature-name>`

Components: `deepstream`, `tao`, `ros2`, `gazebo`, `web`, `api`, `db`, `infra`, `safety`, `real-time`

### Commit Message Format
```
<type>(<component>): <description>

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `safety`: Safety-related changes
- `real-time`: Real-time processing improvements
- `perf`: Performance improvements
- `test`: Testing updates
- `docs`: Documentation updates
- `style`: Code style changes
- `refactor`: Code refactoring
- `build`: Build system changes
- `ci`: CI pipeline updates
- `chore`: Maintenance tasks

### Code Review Process
1. Self-review checklist completion
2. Safety impact assessment
3. Real-time performance validation
4. Automated testing verification
5. Code review by two maintainers
6. Final safety review for critical components

## Contribution Guidelines

### Code Style
- C++: Follow ROS 2 style guide with real-time considerations
- Python: PEP 8 with additional safety-critical guidelines
- Documentation: Clear inline comments for safety-critical sections

### Testing Requirements
1. Unit Tests (90% coverage minimum)
   - Real-time component validation
   - Safety-critical path testing
   - Resource utilization verification

2. Integration Tests
   - Component interaction validation
   - Real-time performance verification
   - Safety system validation
   - Container communication testing

3. Performance Tests
   - Video processing latency (<100ms)
   - Model inference speed (<50ms)
   - Navigation response time (<200ms)
   - Resource utilization benchmarks
   - Network communication latency

### Documentation Requirements
1. Component Documentation
   - Architecture overview
   - Safety considerations
   - Real-time constraints
   - Resource requirements

2. API Documentation
   - Endpoint specifications
   - Safety preconditions
   - Performance characteristics
   - Error handling

### Pull Request Process
1. Create feature branch
2. Implement changes with tests
3. Update documentation
4. Run full test suite
5. Create pull request using template
6. Address review feedback
7. Obtain required approvals
8. Merge after CI/CD validation

## Issue Guidelines

### Bug Reports
Use the bug report template with additional sections:
- Safety Impact Assessment
- Real-time Performance Impact
- System Component Affected
- Reproduction Steps
- Expected vs Actual Behavior

### Feature Requests
Use the feature request template including:
- Safety Considerations
- Performance Requirements
- System Integration Impact
- Resource Requirements
- Testing Strategy

### Security Vulnerabilities
Follow the security disclosure process in SECURITY.md for reporting security issues.

### Issue Labels
- Priority: `P0` (Critical) to `P3` (Low)
- Type: `bug`, `feature`, `safety`, `performance`
- Component: `deepstream`, `tao`, `ros2`, etc.
- Status: `triage`, `investigating`, `in-progress`
- Impact: `safety-critical`, `real-time`, `system-wide`

For detailed information about security-related contributions, please refer to our SECURITY.md document.

Thank you for contributing to the Containerized Autonomous Agricultural Management System. Your adherence to these guidelines helps maintain the safety and reliability of our platform.