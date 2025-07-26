# Getting Started with Constellation Overwatch

Welcome to Constellation Overwatch! This guide will help you install, configure, and run your first autonomous systems coordination platform.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Installation Methods](#installation-methods)
3. [Quick Start](#quick-start)
4. [Configuration](#configuration)
5. [First Steps](#first-steps)
6. [Verification](#verification)
7. [Troubleshooting](#troubleshooting)
8. [Next Steps](#next-steps)

## System Requirements

### Minimum Requirements

| Component | Requirement | Recommended |
|-----------|-------------|-------------|
| **OS** | Ubuntu 20.04+, CentOS 8+, macOS 11+, Windows 10+ | Ubuntu 22.04 LTS |
| **CPU** | 4 cores, 2.0 GHz | 8 cores, 3.0 GHz |
| **RAM** | 8 GB | 16 GB |
| **Storage** | 50 GB available space | 100 GB SSD |
| **Network** | Broadband internet connection | Gigabit ethernet |
| **GPU** | Optional (for AI workloads) | NVIDIA RTX 3060+ |

### Software Dependencies

- **Docker** 20.10+ and Docker Compose 2.0+
- **Python** 3.8+ (for development)
- **Node.js** 16+ (for web interfaces)
- **Git** 2.30+ (for source code)

### Supported Platforms

#### Production Environments
- ✅ **Linux** (Ubuntu, CentOS, RHEL, Debian)
- ✅ **Cloud Platforms** (AWS, Azure, GCP, DigitalOcean)
- ✅ **Kubernetes** (1.21+)
- ✅ **Docker** environments

#### Development Environments
- ✅ **Linux** (All major distributions)
- ✅ **macOS** (Intel and Apple Silicon)
- ✅ **Windows** (with WSL2)

#### Container Platforms
- ✅ **Docker** (Primary deployment method)
- ✅ **Podman** (Alternative container runtime)
- ✅ **Kubernetes** (Orchestrated deployments)
- ✅ **OpenShift** (Enterprise Kubernetes)

## Installation Methods

### Method 1: Docker Compose (Recommended)

This is the fastest way to get Constellation Overwatch running on your system.

#### Step 1: Install Docker

**Linux (Ubuntu/Debian):**
```bash
# Update package index
sudo apt update

# Install Docker
sudo apt install -y docker.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER

# Logout and login again to apply group changes
```

**Linux (CentOS/RHEL):**
```bash
# Install Docker
sudo yum install -y docker docker-compose

# Start and enable Docker
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group
sudo usermod -aG docker $USER
```

**macOS:**
```bash
# Install Docker Desktop
brew install --cask docker

# Start Docker Desktop application
open /Applications/Docker.app
```

**Windows:**
1. Download Docker Desktop from [docker.com](https://www.docker.com/products/docker-desktop/)
2. Install and restart your computer
3. Enable WSL2 integration if prompted

#### Step 2: Download Constellation Overwatch

```bash
# Clone the repository
git clone https://github.com/constellation-overwatch/constellation-overwatch.git
cd constellation-overwatch

# Or download and extract the latest release
curl -L https://github.com/constellation-overwatch/constellation-overwatch/archive/main.zip -o constellation-overwatch.zip
unzip constellation-overwatch.zip
cd constellation-overwatch-main
```

#### Step 3: Start the Services

```bash
# Start all services
docker compose up -d

# Check service status
docker compose ps

# View logs
docker compose logs -f
```

#### Step 4: Verify Installation

Open your browser and navigate to:
- **Web Interface**: http://localhost:8080
- **API Documentation**: http://localhost:8080/docs
- **Admin Panel**: http://localhost:8080/admin

### Method 2: Kubernetes Deployment

For production environments or when you need container orchestration.

#### Prerequisites
- Kubernetes cluster (1.21+)
- kubectl configured
- Helm 3.0+ (optional but recommended)

#### Using Helm (Recommended)

```bash
# Add Constellation Overwatch Helm repository
helm repo add constellation-overwatch https://helm.constellation-overwatch.org
helm repo update

# Install with default values
helm install constellation-overwatch constellation-overwatch/constellation-overwatch

# Or install with custom values
helm install constellation-overwatch constellation-overwatch/constellation-overwatch \
  --set image.tag=latest \
  --set persistence.enabled=true \
  --set ingress.enabled=true \
  --set ingress.hosts[0].host=constellation.example.com
```

#### Using kubectl

```bash
# Apply Kubernetes manifests
kubectl apply -f https://raw.githubusercontent.com/constellation-overwatch/constellation-overwatch/main/deploy/kubernetes/namespace.yaml
kubectl apply -f https://raw.githubusercontent.com/constellation-overwatch/constellation-overwatch/main/deploy/kubernetes/

# Check deployment status
kubectl get pods -n constellation-overwatch
kubectl get services -n constellation-overwatch
```

### Method 3: Binary Installation

For lightweight deployments or when containers aren't available.

#### Download Binaries

```bash
# Linux x64
curl -L https://github.com/constellation-overwatch/constellation-overwatch/releases/latest/download/constellation-overwatch-linux-x64.tar.gz -o constellation-overwatch.tar.gz
tar -xzf constellation-overwatch.tar.gz

# macOS
curl -L https://github.com/constellation-overwatch/constellation-overwatch/releases/latest/download/constellation-overwatch-darwin-x64.tar.gz -o constellation-overwatch.tar.gz
tar -xzf constellation-overwatch.tar.gz

# Windows
curl -L https://github.com/constellation-overwatch/constellation-overwatch/releases/latest/download/constellation-overwatch-windows-x64.zip -o constellation-overwatch.zip
unzip constellation-overwatch.zip
```

#### Install Dependencies

```bash
# Install PostgreSQL
sudo apt install postgresql postgresql-contrib

# Install Redis
sudo apt install redis-server

# Start services
sudo systemctl start postgresql redis-server
sudo systemctl enable postgresql redis-server
```

#### Configure and Run

```bash
# Navigate to installation directory
cd constellation-overwatch

# Copy and edit configuration
cp config/config.example.yaml config/config.yaml
nano config/config.yaml

# Initialize database
./bin/constellation-overwatch migrate

# Start the service
./bin/constellation-overwatch server
```

### Method 4: Source Installation

For developers or when you need to modify the code.

#### Install Development Dependencies

**Python Environment:**
```bash
# Install Python 3.8+
sudo apt install python3 python3-pip python3-venv

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
pip install -r requirements-dev.txt
```

**Node.js Environment:**
```bash
# Install Node.js 16+
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install frontend dependencies
cd frontend
npm install
npm run build
cd ..
```

#### Build from Source

```bash
# Clone repository
git clone https://github.com/constellation-overwatch/constellation-overwatch.git
cd constellation-overwatch

# Install dependencies
make install-deps

# Build the application
make build

# Run tests
make test

# Start development server
make dev
```

## Quick Start

Once you have Constellation Overwatch installed, follow these steps to get up and running quickly.

### 1. Access the Web Interface

Open your browser and go to: http://localhost:8080

You should see the Constellation Overwatch dashboard.

### 2. Initial Setup Wizard

The first time you access the system, you'll be guided through an initial setup:

1. **Create Admin Account**
   - Username: admin
   - Email: admin@example.com
   - Password: Choose a strong password

2. **Configure Basic Settings**
   - System name: "My Constellation System"
   - Time zone: Select your local time zone
   - Units: Metric or Imperial

3. **Set Up First Organization**
   - Organization name: "My Organization"
   - Default location: Your city/region

### 3. Create Your First Entity

Entities represent autonomous systems in your network (drones, robots, sensors, etc.).

```bash
# Using the CLI
constellation-overwatch entity create \
  --id "drone-001" \
  --type "quadcopter" \
  --name "Test Drone 1" \
  --location "39.7392,-104.9903,0"

# Using the API
curl -X POST http://localhost:8080/api/v1/entities \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "id": "drone-001",
    "type": "quadcopter",
    "name": "Test Drone 1",
    "capabilities": ["surveillance", "delivery"],
    "position": {
      "latitude": 39.7392,
      "longitude": -104.9903,
      "altitude": 0.0
    }
  }'
```

### 4. Plan Your First Mission

Create a simple patrol mission:

```bash
# Using the CLI
constellation-overwatch mission create \
  --name "Test Patrol" \
  --type "patrol" \
  --waypoints "39.7392,-104.9903,100|39.7420,-104.9880,100|39.7392,-104.9903,100"

# Using the Web Interface
# 1. Navigate to Missions > Create New
# 2. Select "Patrol" mission type
# 3. Add waypoints on the map
# 4. Assign entities to the mission
# 5. Click "Create Mission"
```

### 5. Monitor Real-Time Status

- **Dashboard**: View overall system status at http://localhost:8080/dashboard
- **Entity Status**: Monitor individual entities at http://localhost:8080/entities
- **Mission Progress**: Track missions at http://localhost:8080/missions
- **Live Map**: See real-time positions at http://localhost:8080/map

## Configuration

### Environment Variables

Key environment variables for configuration:

```bash
# Database Configuration
DATABASE_URL=postgresql://user:password@localhost:5432/constellation
REDIS_URL=redis://localhost:6379/0

# API Configuration
API_HOST=0.0.0.0
API_PORT=8080
API_SECRET_KEY=your-secret-key-here

# Authentication
JWT_SECRET=your-jwt-secret
JWT_EXPIRATION=7200

# Logging
LOG_LEVEL=INFO
LOG_FORMAT=json

# AI/ML Configuration
AI_MODEL_PATH=/app/models
AI_INFERENCE_TIMEOUT=30

# External Services
WEATHER_API_KEY=your-weather-api-key
MAPS_API_KEY=your-maps-api-key
```

### Configuration File

Create a `config.yaml` file for advanced configuration:

```yaml
# config.yaml
server:
  host: "0.0.0.0"
  port: 8080
  debug: false
  
database:
  url: "postgresql://constellation:password@localhost:5432/constellation"
  pool_size: 20
  max_overflow: 30
  
redis:
  url: "redis://localhost:6379/0"
  max_connections: 100
  
authentication:
  jwt_secret: "your-jwt-secret"
  jwt_expiration: 7200
  session_timeout: 3600
  
ai_engine:
  model_path: "/app/models"
  inference_timeout: 30
  max_concurrent_requests: 10
  
logging:
  level: "INFO"
  format: "json"
  output: "stdout"
  
features:
  ai_powered_missions: true
  real_time_tracking: true
  weather_integration: true
  terrain_analysis: true
```

### Docker Compose Configuration

Customize your `docker-compose.yml`:

```yaml
version: '3.8'

services:
  constellation-api:
    image: constellation-overwatch/api:latest
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgresql://postgres:password@db:5432/constellation
      - REDIS_URL=redis://redis:6379/0
    depends_on:
      - db
      - redis
    volumes:
      - ./config:/app/config
      - ./logs:/app/logs
      
  constellation-worker:
    image: constellation-overwatch/worker:latest
    environment:
      - DATABASE_URL=postgresql://postgres:password@db:5432/constellation
      - REDIS_URL=redis://redis:6379/0
    depends_on:
      - db
      - redis
      
  db:
    image: postgres:13
    environment:
      - POSTGRES_DB=constellation
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=password
    volumes:
      - postgres_data:/var/lib/postgresql/data
      
  redis:
    image: redis:7-alpine
    volumes:
      - redis_data:/data

volumes:
  postgres_data:
  redis_data:
```

## First Steps

### 1. Explore the Dashboard

Visit http://localhost:8080 to see the main dashboard:

- **System Overview**: Current status and key metrics
- **Entity Map**: Real-time positions of all entities
- **Active Missions**: Currently running missions
- **Alerts**: System alerts and notifications
- **Performance Metrics**: System health indicators

### 2. Add Your First Entities

Start by adding entities that represent your autonomous systems:

**For Drones:**
```json
{
  "id": "drone-001",
  "type": "quadcopter",
  "name": "Surveillance Drone Alpha",
  "capabilities": ["surveillance", "photography", "delivery"],
  "specifications": {
    "max_altitude": 400,
    "max_speed": 15,
    "battery_capacity": 5000,
    "payload_capacity": 2.0
  },
  "position": {
    "latitude": 39.7392,
    "longitude": -104.9903,
    "altitude": 0.0
  }
}
```

**For Ground Robots:**
```json
{
  "id": "robot-001",
  "type": "ground_robot",
  "name": "Inspection Robot Beta",
  "capabilities": ["inspection", "mapping", "sample_collection"],
  "specifications": {
    "max_speed": 5,
    "battery_capacity": 10000,
    "ground_clearance": 0.15
  },
  "position": {
    "latitude": 39.7420,
    "longitude": -104.9880,
    "altitude": 0.0
  }
}
```

**For Sensors:**
```json
{
  "id": "sensor-001",
  "type": "weather_station",
  "name": "Weather Station Gamma",
  "capabilities": ["temperature", "humidity", "wind_speed", "air_pressure"],
  "specifications": {
    "measurement_interval": 60,
    "data_retention": 2592000
  },
  "position": {
    "latitude": 39.7400,
    "longitude": -104.9890,
    "altitude": 1650.0
  }
}
```

### 3. Create Your First Mission

Try different mission types:

**Patrol Mission:**
```python
from constellation_overwatch import ConstellationClient

client = ConstellationClient(api_key="your-api-key")

mission = client.missions.create({
    "name": "Security Patrol Alpha",
    "type": "patrol",
    "description": "Regular security patrol of the perimeter",
    "waypoints": [
        {"latitude": 39.7392, "longitude": -104.9903, "altitude": 50, "action": "patrol"},
        {"latitude": 39.7420, "longitude": -104.9903, "altitude": 50, "action": "patrol"},
        {"latitude": 39.7420, "longitude": -104.9880, "altitude": 50, "action": "patrol"},
        {"latitude": 39.7392, "longitude": -104.9880, "altitude": 50, "action": "patrol"},
        {"latitude": 39.7392, "longitude": -104.9903, "altitude": 50, "action": "land"}
    ],
    "schedule": {
        "type": "recurring",
        "interval": "daily",
        "time": "06:00"
    }
})

print(f"Mission created: {mission.id}")
```

**Search and Rescue Mission:**
```python
search_mission = client.missions.create({
    "name": "Search and Rescue Operation",
    "type": "search",
    "description": "Search for missing person in designated area",
    "search_area": {
        "type": "polygon",
        "coordinates": [
            [39.7390, -104.9905],
            [39.7430, -104.9905],
            [39.7430, -104.9875],
            [39.7390, -104.9875],
            [39.7390, -104.9905]
        ]
    },
    "search_pattern": "grid",
    "altitude": 75,
    "overlap": 30,
    "priority": "high"
})
```

### 4. Set Up Real-Time Monitoring

Enable real-time updates via WebSocket:

```javascript
// Connect to WebSocket
const ws = new WebSocket('ws://localhost:8080/api/v1/ws');

// Authenticate
ws.onopen = function() {
    ws.send(JSON.stringify({
        type: 'auth',
        token: 'your-api-key'
    }));
};

// Subscribe to entity updates
ws.send(JSON.stringify({
    type: 'subscribe',
    channel: 'entities'
}));

// Handle updates
ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    
    if (data.type === 'entity_update') {
        console.log('Entity update:', data.entity);
        updateEntityOnMap(data.entity);
    }
    
    if (data.type === 'mission_event') {
        console.log('Mission event:', data.event);
        updateMissionStatus(data.mission_id, data.event);
    }
};
```

### 5. Configure Integrations

Set up integrations with external systems:

**Weather Integration:**
```bash
# Set weather API key
export WEATHER_API_KEY="your-openweathermap-api-key"

# Test weather integration
curl -X GET "http://localhost:8080/api/v1/weather?lat=39.7392&lon=-104.9903"
```

**ROS Integration:**
```bash
# Install ROS bridge
pip install constellation-overwatch-ros

# Configure ROS connection
roslaunch constellation_overwatch_ros bridge.launch \
  constellation_host:=localhost \
  constellation_port:=8080 \
  api_key:=your-api-key
```

## Verification

### Health Checks

Verify your installation is working correctly:

```bash
# Check system health
curl http://localhost:8080/api/v1/health

# Expected response:
{
  "status": "healthy",
  "version": "1.0.0",
  "services": {
    "database": "healthy",
    "redis": "healthy",
    "ai_engine": "healthy"
  },
  "timestamp": "2025-01-30T16:00:00Z"
}
```

### API Tests

Test core API functionality:

```bash
# Create API key
curl -X POST http://localhost:8080/api/v1/auth/keys \
  -H "Content-Type: application/json" \
  -d '{"name": "test-key", "permissions": ["read", "write"]}'

# Test entity creation
curl -X POST http://localhost:8080/api/v1/entities \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -d '{
    "id": "test-entity",
    "type": "test",
    "name": "Test Entity"
  }'

# Test entity retrieval
curl -X GET http://localhost:8080/api/v1/entities/test-entity \
  -H "Authorization: Bearer YOUR_API_KEY"
```

### Performance Tests

Run basic performance tests:

```bash
# Install testing tools
pip install locust

# Run load test
locust -f tests/load/basic_api_test.py --host=http://localhost:8080
```

### Integration Tests

Verify integrations are working:

```bash
# Run integration test suite
./scripts/run-integration-tests.sh

# Test specific integration
./scripts/test-weather-integration.sh
./scripts/test-ros-integration.sh
```

## Troubleshooting

### Common Issues

#### 1. Services Not Starting

**Problem**: Docker containers fail to start

**Solution**:
```bash
# Check Docker status
sudo systemctl status docker

# Check container logs
docker compose logs constellation-api
docker compose logs db

# Restart services
docker compose down
docker compose up -d
```

#### 2. Database Connection Issues

**Problem**: Cannot connect to database

**Solution**:
```bash
# Check PostgreSQL status
docker compose exec db pg_isready

# Reset database
docker compose down -v
docker compose up -d

# Check database logs
docker compose logs db
```

#### 3. Permission Errors

**Problem**: Permission denied errors

**Solution**:
```bash
# Fix file permissions
sudo chown -R $USER:$USER .
chmod +x scripts/*.sh

# Fix Docker permissions
sudo usermod -aG docker $USER
# Logout and login again
```

#### 4. API Authentication Failures

**Problem**: API returns 401 Unauthorized

**Solution**:
```bash
# Check API key
curl -X GET http://localhost:8080/api/v1/auth/verify \
  -H "Authorization: Bearer YOUR_API_KEY"

# Generate new API key
curl -X POST http://localhost:8080/api/v1/auth/keys \
  -H "Content-Type: application/json" \
  -d '{"name": "new-key"}'
```

#### 5. WebSocket Connection Issues

**Problem**: Real-time updates not working

**Solution**:
```javascript
// Check WebSocket connection
const ws = new WebSocket('ws://localhost:8080/api/v1/ws');

ws.onerror = function(error) {
    console.error('WebSocket error:', error);
};

ws.onclose = function(event) {
    console.log('WebSocket closed:', event.code, event.reason);
};

// Enable debug logging
localStorage.setItem('constellation_debug', 'true');
```

### Getting Help

If you encounter issues not covered here:

1. **Check the logs**:
   ```bash
   docker compose logs -f
   tail -f logs/constellation.log
   ```

2. **Search existing issues**:
   - [GitHub Issues](https://github.com/constellation-overwatch/constellation-overwatch/issues)
   - [Community Forum](https://community.constellation-overwatch.org)

3. **Ask for help**:
   - [Discord Community](https://constellation-overwatch.discord.gg)
   - [Stack Overflow](https://stackoverflow.com/questions/tagged/constellation-overwatch)

4. **Report bugs**:
   - [Submit an Issue](https://github.com/constellation-overwatch/constellation-overwatch/issues/new)
   - Include system info, logs, and reproduction steps

## Next Steps

Now that you have Constellation Overwatch running, here's what to explore next:

### 🎓 Learning Resources
- [Basic Concepts](../concepts/overview.md) - Understand core concepts
- [Quick Start Tutorial](../tutorials/quick-start.md) - Build your first application
- [API Documentation](../api/overview.md) - Learn the API

### 🔧 Advanced Configuration
- [Production Deployment](../guides/production-deployment.md) - Deploy to production
- [Security Configuration](../guides/security-setup.md) - Secure your installation
- [Performance Tuning](../guides/performance-tuning.md) - Optimize performance

### 🌐 Integration Guides
- [ROS Integration](../examples/ros-integration.md) - Connect with ROS systems
- [Cloud Integration](../examples/cloud-integration.md) - Deploy to cloud platforms
- [External API Integration](../examples/api-integration.md) - Connect external systems

### 🏗️ Development
- [Development Setup](../development/environment-setup.md) - Set up development environment
- [Plugin Development](../development/plugins.md) - Create custom plugins
- [Contributing Guide](../../COMMUNITY_GUIDELINES.md) - Contribute to the project

### 🌟 Community
- [Join Discord](https://constellation-overwatch.discord.gg) - Connect with the community
- [Community Guidelines](../../COMMUNITY_GUIDELINES.md) - Learn community standards
- [Events Calendar](../help/events.md) - Upcoming events and workshops

---

**Congratulations!** You now have Constellation Overwatch up and running. Welcome to the community of autonomous systems builders!

If you found this guide helpful, please consider:
- ⭐ Starring the [repository](https://github.com/constellation-overwatch/constellation-overwatch)
- 📢 Sharing your experience in our [Discord community](https://constellation-overwatch.discord.gg)
- 📝 Contributing improvements to this documentation

---

**Need Help?** Join our [Discord community](https://constellation-overwatch.discord.gg) or check our [troubleshooting guide](../help/troubleshooting.md).

**Last Updated**: January 30, 2025