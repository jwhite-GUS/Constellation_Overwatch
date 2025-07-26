# Constellation Overwatch SDK - API Documentation

**Version**: 1.0.0  
**Last Updated**: January 30, 2025  
**Purpose**: Comprehensive API reference for the Constellation Overwatch SDK

## Table of Contents

1. [Overview](#overview)
2. [Authentication](#authentication)
3. [Core API Reference](#core-api-reference)
4. [SDK Libraries](#sdk-libraries)
5. [Examples](#examples)
6. [Error Handling](#error-handling)
7. [Rate Limits](#rate-limits)
8. [Versioning](#versioning)

## Overview

The Constellation Overwatch SDK provides a comprehensive API for autonomous systems integration, AI-powered decision making, and real-time command and control operations. This documentation covers all available endpoints, methods, and integration patterns.

### Base URL
```
Production: https://api.constellation-overwatch.mil/v1
Development: http://localhost:8080/api/v1
```

### Supported Formats
- **Request**: JSON, form-data (for file uploads)
- **Response**: JSON
- **Websocket**: Real-time communication

### API Architecture
The API follows REST principles with real-time capabilities through WebSocket connections:

```
┌─────────────────────────────────────────┐
│            Client Applications          │
├─────────────────────────────────────────┤
│              SDK Libraries              │
├─────────────────────────────────────────┤
│         HTTP REST API / WebSocket       │
├─────────────────────────────────────────┤
│            API Gateway Layer           │
├─────────────────────────────────────────┤
│           Core Services                 │
└─────────────────────────────────────────┘
```

## Authentication

### API Key Authentication

All API requests require authentication using API keys. Include your API key in the request header:

```http
Authorization: Bearer YOUR_API_KEY
```

### Obtaining API Keys

1. **Development Environment**:
   ```bash
   # Generate development API key
   curl -X POST http://localhost:8080/api/v1/auth/keys \
     -H "Content-Type: application/json" \
     -d '{"name": "dev-key", "scope": ["read", "write"]}'
   ```

2. **Production Environment**:
   - Contact your system administrator
   - Use PKI certificates for enhanced security
   - Follow your organization's key management policies

### Token-Based Authentication (OAuth 2.0)

For user-based authentication, the API supports OAuth 2.0 flows:

```http
POST /api/v1/auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "constellation.read constellation.write"
}
```

**Response**:
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "constellation.read constellation.write"
}
```

## Core API Reference

### Entity Management

#### List Entities
Retrieve all entities in the system with optional filtering.

```http
GET /api/v1/entities
```

**Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `type` | string | No | Filter by entity type (drone, sensor, ground_station) |
| `status` | string | No | Filter by status (active, inactive, maintenance) |
| `limit` | integer | No | Maximum number of results (default: 100, max: 1000) |
| `offset` | integer | No | Number of results to skip (default: 0) |

**Example Request**:
```bash
curl -X GET "https://api.constellation-overwatch.mil/v1/entities?type=drone&status=active&limit=50" \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Accept: application/json"
```

**Example Response**:
```json
{
  "data": [
    {
      "id": "drone_001",
      "type": "drone",
      "status": "active",
      "position": {
        "latitude": 39.7392,
        "longitude": -104.9903,
        "altitude": 100.0
      },
      "capabilities": ["surveillance", "reconnaissance"],
      "battery_level": 85,
      "last_contact": "2025-01-30T15:30:00Z",
      "metadata": {
        "model": "Quadcopter-X1",
        "firmware_version": "2.1.3"
      }
    }
  ],
  "pagination": {
    "total": 1,
    "limit": 50,
    "offset": 0,
    "has_more": false
  }
}
```

#### Create Entity
Register a new entity in the system.

```http
POST /api/v1/entities
```

**Request Body**:
```json
{
  "id": "drone_002",
  "type": "drone",
  "capabilities": ["surveillance", "delivery"],
  "position": {
    "latitude": 40.7128,
    "longitude": -74.0060,
    "altitude": 0.0
  },
  "metadata": {
    "model": "Delivery-Drone-V2",
    "max_payload": "5kg",
    "range": "25km"
  }
}
```

**Response**:
```json
{
  "id": "drone_002",
  "type": "drone",
  "status": "inactive",
  "created_at": "2025-01-30T16:00:00Z",
  "message": "Entity created successfully"
}
```

#### Get Entity Details
Retrieve detailed information about a specific entity.

```http
GET /api/v1/entities/{entity_id}
```

**Path Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `entity_id` | string | Yes | Unique identifier for the entity |

**Example Response**:
```json
{
  "id": "drone_001",
  "type": "drone",
  "status": "active",
  "position": {
    "latitude": 39.7392,
    "longitude": -104.9903,
    "altitude": 100.0,
    "heading": 270.5,
    "speed": 15.2
  },
  "capabilities": ["surveillance", "reconnaissance"],
  "battery_level": 85,
  "last_contact": "2025-01-30T15:30:00Z",
  "mission_status": "in_progress",
  "components": [
    {
      "type": "camera",
      "status": "operational",
      "settings": {
        "resolution": "4K",
        "zoom": "2x"
      }
    }
  ],
  "metadata": {
    "model": "Quadcopter-X1",
    "firmware_version": "2.1.3",
    "serial_number": "QX1-2024-001"
  }
}
```

#### Update Entity
Update an existing entity's properties.

```http
PUT /api/v1/entities/{entity_id}
```

**Request Body**:
```json
{
  "status": "maintenance",
  "position": {
    "latitude": 39.7392,
    "longitude": -104.9903,
    "altitude": 0.0
  },
  "metadata": {
    "last_maintenance": "2025-01-30T16:00:00Z"
  }
}
```

#### Delete Entity
Remove an entity from the system.

```http
DELETE /api/v1/entities/{entity_id}
```

**Response**:
```json
{
  "message": "Entity deleted successfully",
  "deleted_at": "2025-01-30T16:15:00Z"
}
```

### Mission Management

#### List Missions
Retrieve all missions with optional filtering.

```http
GET /api/v1/missions
```

**Parameters**:
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `status` | string | No | Filter by status (planning, active, completed, failed) |
| `entity_id` | string | No | Filter by assigned entity |
| `mission_type` | string | No | Filter by mission type |

**Example Response**:
```json
{
  "data": [
    {
      "id": "mission_001",
      "name": "Surveillance Route Alpha",
      "type": "surveillance",
      "status": "active",
      "assigned_entities": ["drone_001"],
      "waypoints": [
        {
          "latitude": 39.7392,
          "longitude": -104.9903,
          "altitude": 100.0,
          "action": "survey"
        }
      ],
      "created_at": "2025-01-30T14:00:00Z",
      "started_at": "2025-01-30T15:00:00Z",
      "estimated_completion": "2025-01-30T17:00:00Z"
    }
  ]
}
```

#### Create Mission
Create a new mission plan.

```http
POST /api/v1/missions
```

**Request Body**:
```json
{
  "name": "Emergency Response Alpha",
  "type": "emergency_response",
  "priority": "high",
  "waypoints": [
    {
      "latitude": 40.7128,
      "longitude": -74.0060,
      "altitude": 150.0,
      "action": "search",
      "duration": 300
    }
  ],
  "parameters": {
    "search_pattern": "grid",
    "coverage_area": 1000,
    "image_interval": 5
  }
}
```

#### Start Mission
Begin execution of a planned mission.

```http
POST /api/v1/missions/{mission_id}/start
```

**Request Body**:
```json
{
  "assigned_entities": ["drone_001", "drone_002"],
  "start_time": "immediate"
}
```

### Real-Time Communication

#### WebSocket Connection
Establish real-time communication for telemetry and commands.

```javascript
const ws = new WebSocket('wss://api.constellation-overwatch.mil/v1/ws');

// Authentication
ws.onopen = function() {
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'YOUR_API_KEY'
  }));
};

// Subscribe to entity updates
ws.send(JSON.stringify({
  type: 'subscribe',
  channel: 'entity.drone_001.telemetry'
}));

// Receive real-time updates
ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  console.log('Received:', data);
};
```

**Message Types**:
- `telemetry`: Real-time entity status updates
- `command`: Send commands to entities
- `event`: System events and notifications
- `heartbeat`: Connection keep-alive

### AI Services

#### Object Detection
Analyze images using AI-powered object detection.

```http
POST /api/v1/ai/vision/detect
```

**Request Body** (multipart/form-data):
```
image: [binary image data]
model: "yolo_v5"
confidence_threshold: 0.7
```

**Response**:
```json
{
  "detections": [
    {
      "class": "vehicle",
      "confidence": 0.95,
      "bbox": {
        "x": 100,
        "y": 150,
        "width": 200,
        "height": 100
      }
    }
  ],
  "processing_time": 0.234,
  "model_version": "yolo_v5.2"
}
```

#### Natural Language Commands
Process natural language commands for mission planning.

```http
POST /api/v1/ai/nlp/command
```

**Request Body**:
```json
{
  "text": "Send drone to investigate the area around coordinates 40.7128, -74.0060 at altitude 200 feet",
  "context": {
    "available_entities": ["drone_001", "drone_002"],
    "current_missions": ["mission_001"]
  }
}
```

**Response**:
```json
{
  "intent": "create_mission",
  "entities": {
    "location": {
      "latitude": 40.7128,
      "longitude": -74.0060
    },
    "altitude": 200,
    "action": "investigate"
  },
  "confidence": 0.89,
  "suggested_mission": {
    "type": "investigation",
    "waypoints": [
      {
        "latitude": 40.7128,
        "longitude": -74.0060,
        "altitude": 60.96,
        "action": "investigate"
      }
    ]
  }
}
```

## SDK Libraries

The Constellation Overwatch SDK provides client libraries for multiple programming languages:

### Python SDK

#### Installation
```bash
pip install constellation-overwatch-sdk
```

#### Quick Start
```python
from constellation_overwatch import ConstellationClient

# Initialize client
client = ConstellationClient(
    api_key="YOUR_API_KEY",
    base_url="https://api.constellation-overwatch.mil/v1"
)

# List all active drones
drones = client.entities.list(type="drone", status="active")
print(f"Found {len(drones)} active drones")

# Get specific drone details
drone = client.entities.get("drone_001")
print(f"Drone position: {drone.position}")

# Create and start a mission
mission = client.missions.create({
    "name": "Patrol Route Charlie",
    "type": "patrol",
    "waypoints": [
        {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0}
    ]
})

client.missions.start(mission.id, assigned_entities=["drone_001"])
```

#### Real-time Telemetry
```python
import asyncio
from constellation_overwatch import ConstellationWebSocket

async def handle_telemetry():
    ws = ConstellationWebSocket("YOUR_API_KEY")
    
    # Subscribe to drone telemetry
    await ws.subscribe("entity.drone_001.telemetry")
    
    async for message in ws.listen():
        if message.type == "telemetry":
            print(f"Drone position: {message.data.position}")
            print(f"Battery level: {message.data.battery_level}%")

asyncio.run(handle_telemetry())
```

### JavaScript/Node.js SDK

#### Installation
```bash
npm install @constellation-overwatch/sdk
```

#### Quick Start
```javascript
const { ConstellationClient } = require('@constellation-overwatch/sdk');

const client = new ConstellationClient({
  apiKey: 'YOUR_API_KEY',
  baseURL: 'https://api.constellation-overwatch.mil/v1'
});

// List entities
async function listDrones() {
  try {
    const drones = await client.entities.list({
      type: 'drone',
      status: 'active'
    });
    console.log(`Found ${drones.length} active drones`);
  } catch (error) {
    console.error('Error listing drones:', error);
  }
}

// Real-time updates
const ws = client.websocket();
ws.on('connect', () => {
  ws.subscribe('entity.*.telemetry');
});

ws.on('telemetry', (data) => {
  console.log(`Entity ${data.entity_id} telemetry:`, data);
});
```

### C++ SDK

#### Installation
```cmake
find_package(ConstellationOverwatch REQUIRED)
target_link_libraries(your_target ConstellationOverwatch::SDK)
```

#### Quick Start
```cpp
#include <constellation_overwatch/client.h>

int main() {
    constellation::Client client("YOUR_API_KEY");
    
    // List active drones
    auto drones = client.entities().list()
        .type("drone")
        .status("active")
        .execute();
    
    std::cout << "Found " << drones.size() << " active drones\n";
    
    // Get drone details
    auto drone = client.entities().get("drone_001");
    std::cout << "Drone position: " 
              << drone.position().latitude() << ", "
              << drone.position().longitude() << "\n";
    
    return 0;
}
```

## Examples

### Complete Mission Workflow

This example demonstrates a complete workflow from entity registration to mission execution:

```python
from constellation_overwatch import ConstellationClient
import time

# Initialize client
client = ConstellationClient(api_key="YOUR_API_KEY")

# 1. Register a new drone
drone_data = {
    "id": "patrol_drone_01",
    "type": "drone",
    "capabilities": ["surveillance", "patrol"],
    "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0},
    "metadata": {
        "model": "Patrol-Drone-Pro",
        "max_flight_time": "45min",
        "camera_specs": "4K with thermal imaging"
    }
}

drone = client.entities.create(drone_data)
print(f"Registered drone: {drone.id}")

# 2. Wait for drone to come online
while True:
    drone_status = client.entities.get(drone.id)
    if drone_status.status == "active":
        print("Drone is online and ready")
        break
    time.sleep(5)

# 3. Create a patrol mission
mission_data = {
    "name": "Downtown Patrol Route",
    "type": "patrol",
    "priority": "normal",
    "waypoints": [
        {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0, "action": "patrol"},
        {"latitude": 39.7420, "longitude": -104.9880, "altitude": 100.0, "action": "patrol"},
        {"latitude": 39.7450, "longitude": -104.9910, "altitude": 100.0, "action": "patrol"},
        {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0, "action": "land"}
    ],
    "parameters": {
        "patrol_speed": 10.0,
        "camera_recording": True,
        "return_to_base": True
    }
}

mission = client.missions.create(mission_data)
print(f"Created mission: {mission.id}")

# 4. Start the mission
client.missions.start(mission.id, assigned_entities=[drone.id])
print("Mission started")

# 5. Monitor mission progress
while True:
    mission_status = client.missions.get(mission.id)
    drone_status = client.entities.get(drone.id)
    
    print(f"Mission status: {mission_status.status}")
    print(f"Drone position: {drone_status.position}")
    print(f"Battery level: {drone_status.battery_level}%")
    
    if mission_status.status in ["completed", "failed"]:
        print(f"Mission {mission_status.status}")
        break
    
    time.sleep(10)
```

### AI-Powered Search and Rescue

```python
import asyncio
from constellation_overwatch import ConstellationClient

async def search_and_rescue_mission():
    client = ConstellationClient(api_key="YOUR_API_KEY")
    
    # 1. Define search area
    search_area = {
        "center": {"latitude": 40.7128, "longitude": -74.0060},
        "radius": 2000,  # meters
        "pattern": "expanding_square"
    }
    
    # 2. Find available drones with camera capabilities
    available_drones = client.entities.list(
        type="drone",
        status="active",
        capabilities=["camera", "thermal_imaging"]
    )
    
    if not available_drones:
        print("No suitable drones available")
        return
    
    # 3. Create search mission with AI analysis
    mission_data = {
        "name": "Search and Rescue Operation",
        "type": "search_rescue",
        "priority": "high",
        "search_area": search_area,
        "ai_analysis": {
            "detect_people": True,
            "detect_vehicles": True,
            "thermal_analysis": True,
            "alert_on_detection": True
        }
    }
    
    mission = client.missions.create(mission_data)
    
    # 4. Start mission with multiple drones
    drone_ids = [drone.id for drone in available_drones[:3]]  # Use up to 3 drones
    client.missions.start(mission.id, assigned_entities=drone_ids)
    
    print(f"Search and rescue mission started with {len(drone_ids)} drones")
    
    # 5. Process real-time AI detections
    ws = client.websocket()
    await ws.subscribe(f"mission.{mission.id}.ai_detections")
    
    async for message in ws.listen():
        if message.type == "ai_detection":
            detection = message.data
            if detection.class in ["person", "vehicle"]:
                print(f"ALERT: {detection.class} detected at {detection.location}")
                print(f"Confidence: {detection.confidence}")
                
                # Automatically create investigation task
                investigation_data = {
                    "name": f"Investigate {detection.class}",
                    "type": "investigation",
                    "priority": "urgent",
                    "target_location": detection.location,
                    "investigation_duration": 300  # 5 minutes
                }
                
                investigation = client.missions.create(investigation_data)
                print(f"Created investigation mission: {investigation.id}")

# Run the async function
asyncio.run(search_and_rescue_mission())
```

## Error Handling

The API uses standard HTTP status codes and provides detailed error information:

### HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 409 | Conflict |
| 429 | Rate Limited |
| 500 | Internal Server Error |
| 503 | Service Unavailable |

### Error Response Format

```json
{
  "error": {
    "code": "INVALID_ENTITY_TYPE",
    "message": "The specified entity type 'invalid_type' is not supported",
    "details": {
      "field": "type",
      "provided_value": "invalid_type",
      "allowed_values": ["drone", "sensor", "ground_station"]
    },
    "request_id": "req_12345",
    "timestamp": "2025-01-30T16:30:00Z"
  }
}
```

### Common Error Codes

| Code | Description | Resolution |
|------|-------------|------------|
| `INVALID_API_KEY` | API key is missing or invalid | Check your API key configuration |
| `INSUFFICIENT_PERMISSIONS` | API key lacks required permissions | Contact administrator for access |
| `ENTITY_NOT_FOUND` | Requested entity doesn't exist | Verify entity ID is correct |
| `MISSION_CONFLICT` | Entity already assigned to another mission | Complete or cancel existing mission |
| `VALIDATION_ERROR` | Request data validation failed | Check required fields and data types |
| `RATE_LIMIT_EXCEEDED` | Too many requests | Implement exponential backoff |

### SDK Error Handling Examples

#### Python
```python
from constellation_overwatch import ConstellationClient, ConstellationError

client = ConstellationClient(api_key="YOUR_API_KEY")

try:
    drone = client.entities.get("nonexistent_drone")
except ConstellationError as e:
    if e.error_code == "ENTITY_NOT_FOUND":
        print("Drone not found, creating new drone...")
        # Handle missing entity
    else:
        print(f"Unexpected error: {e.message}")
        raise
```

#### JavaScript
```javascript
const { ConstellationClient, ConstellationError } = require('@constellation-overwatch/sdk');

const client = new ConstellationClient({apiKey: 'YOUR_API_KEY'});

try {
  const drone = await client.entities.get('nonexistent_drone');
} catch (error) {
  if (error instanceof ConstellationError) {
    console.log(`API Error: ${error.code} - ${error.message}`);
  } else {
    console.log(`Network Error: ${error.message}`);
  }
}
```

## Rate Limits

To ensure fair usage and system stability, the API implements rate limiting:

### Limits by Plan

| Plan | Requests/Minute | Concurrent Connections | Burst Limit |
|------|----------------|----------------------|-------------|
| Development | 100 | 10 | 200 |
| Professional | 1,000 | 50 | 2,000 |
| Enterprise | 10,000 | 200 | 20,000 |
| Government | Unlimited | 1,000 | Unlimited |

### Rate Limit Headers

Every API response includes rate limit information:

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1643723400
X-RateLimit-Burst: 2000
```

### Handling Rate Limits

Implement exponential backoff when rate limits are exceeded:

```python
import time
import random
from constellation_overwatch import ConstellationClient, RateLimitError

def make_request_with_backoff(func, max_retries=5):
    for attempt in range(max_retries):
        try:
            return func()
        except RateLimitError as e:
            if attempt == max_retries - 1:
                raise
            
            # Exponential backoff with jitter
            delay = (2 ** attempt) + random.uniform(0, 1)
            time.sleep(delay)
    
    raise Exception("Max retries exceeded")

# Usage
client = ConstellationClient(api_key="YOUR_API_KEY")
drones = make_request_with_backoff(
    lambda: client.entities.list(type="drone")
)
```

## Versioning

The API uses semantic versioning with backwards compatibility guarantees:

### Version Format
- **Major.Minor.Patch** (e.g., 1.2.3)
- **Major**: Breaking changes
- **Minor**: New features, backwards compatible
- **Patch**: Bug fixes, backwards compatible

### API Versioning Strategy

1. **URL Versioning**: Version specified in URL path
   ```
   /api/v1/entities
   /api/v2/entities
   ```

2. **Header Versioning**: Version specified in request header
   ```http
   API-Version: 1.2
   ```

3. **Backwards Compatibility**: Maintained for at least 12 months

### Migration Guides

When new major versions are released, migration guides are provided:

- **Breaking Changes**: Detailed list with alternatives
- **Deprecated Features**: Timeline for removal
- **New Features**: Usage examples and benefits
- **SDK Updates**: Updated library versions

### Current API Versions

| Version | Status | Support End Date | Notes |
|---------|--------|-----------------|-------|
| v1.0 | Current | Active | Stable release |
| v0.9 | Deprecated | 2025-06-30 | Legacy support |
| v2.0 | Beta | TBD | Preview features |

---

*This API documentation is continuously updated. For the latest information, visit our [developer portal](https://developers.constellation-overwatch.mil) or subscribe to our [API changelog](https://api.constellation-overwatch.mil/changelog).*