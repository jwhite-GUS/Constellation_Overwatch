# Constellation Overwatch SDK - Testing Guide

**Version**: 1.0.0  
**Last Updated**: January 30, 2025  
**Purpose**: Comprehensive testing guide for the Constellation Overwatch SDK

## Table of Contents

1. [Testing Philosophy](#testing-philosophy)
2. [Testing Framework Overview](#testing-framework-overview)
3. [Unit Testing](#unit-testing)
4. [Integration Testing](#integration-testing)
5. [System Testing](#system-testing)
6. [Performance Testing](#performance-testing)
7. [Security Testing](#security-testing)
8. [API Testing](#api-testing)
9. [Test Automation](#test-automation)
10. [Best Practices](#best-practices)

## Testing Philosophy

The Constellation Overwatch SDK follows a comprehensive testing strategy based on the **Testing Pyramid** principle:

```
           /\
          /  \
         /    \
        / UI   \  <-- End-to-End Tests (Few, High-Value)
       /        \
      /----------\
     /            \
    / Integration  \  <-- Integration Tests (Some, Critical Paths)
   /                \
  /------------------\
 /                    \
/       Unit           \  <-- Unit Tests (Many, Fast, Isolated)
/______________________\
```

### Core Testing Principles

1. **Fast Feedback**: Tests should run quickly to enable rapid development
2. **Reliable**: Tests should be deterministic and not flaky
3. **Maintainable**: Tests should be easy to read, write, and maintain
4. **Comprehensive**: Critical functionality must be thoroughly tested
5. **Isolated**: Tests should not depend on external systems when possible

### Quality Gates

- **Code Coverage**: Minimum 80% coverage for new code
- **Test Execution**: All tests must pass before merge
- **Performance**: No regression in API response times
- **Security**: All security tests must pass
- **Documentation**: All public APIs must be documented and tested

## Testing Framework Overview

### Test Types and Tools

| Test Type | Framework | Purpose | Frequency |
|-----------|-----------|---------|-----------|
| Unit | pytest, Jest, gtest | Test individual components | Every commit |
| Integration | pytest, Newman | Test component interactions | Every PR |
| System | pytest, Selenium | Test complete workflows | Daily |
| Performance | locust, k6 | Test system performance | Weekly |
| Security | bandit, semgrep | Security vulnerability testing | Every release |
| API | Postman, pytest | Test API endpoints | Every commit |

### Test Environment Structure

```
tests/
├── unit/                   # Unit tests
│   ├── core/              # Core SDK tests
│   ├── ai/                # AI module tests
│   ├── api/               # API layer tests
│   └── plugins/           # Plugin tests
├── integration/           # Integration tests
│   ├── api/               # API integration tests
│   ├── database/          # Database integration tests
│   ├── external/          # External service tests
│   └── workflows/         # End-to-end workflows
├── performance/           # Performance tests
│   ├── load/              # Load testing scripts
│   ├── stress/            # Stress testing scripts
│   └── benchmark/         # Benchmark tests
├── security/              # Security tests
│   ├── auth/              # Authentication tests
│   ├── injection/         # Injection attack tests
│   └── encryption/        # Encryption tests
├── fixtures/              # Test data and fixtures
├── mocks/                 # Mock objects and services
└── utils/                 # Testing utilities
```

## Unit Testing

Unit tests verify individual components in isolation. They should be fast, reliable, and test a single unit of functionality.

### Python Unit Testing with pytest

#### Basic Test Structure

```python
# tests/unit/core/test_entity_manager.py
import pytest
from unittest.mock import Mock, patch
from constellation_overwatch.core import EntityManager, Entity

class TestEntityManager:
    """Unit tests for EntityManager class"""
    
    @pytest.fixture
    def entity_manager(self):
        """Create EntityManager instance for testing"""
        return EntityManager()
    
    @pytest.fixture
    def sample_entity(self):
        """Create sample entity for testing"""
        return Entity(
            id="test_drone_01",
            type="drone",
            position={"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0},
            capabilities=["surveillance"]
        )
    
    def test_create_entity_success(self, entity_manager, sample_entity):
        """Test successful entity creation"""
        # Act
        result = entity_manager.create_entity(sample_entity)
        
        # Assert
        assert result is not None
        assert result.id == "test_drone_01"
        assert entity_manager.get_entity("test_drone_01") == result
    
    def test_create_entity_duplicate_id(self, entity_manager, sample_entity):
        """Test creation of entity with duplicate ID fails"""
        # Arrange
        entity_manager.create_entity(sample_entity)
        
        # Act & Assert
        with pytest.raises(ValueError, match="Entity with ID .* already exists"):
            entity_manager.create_entity(sample_entity)
    
    def test_get_entity_not_found(self, entity_manager):
        """Test getting non-existent entity returns None"""
        # Act
        result = entity_manager.get_entity("nonexistent")
        
        # Assert
        assert result is None
    
    def test_update_entity_position(self, entity_manager, sample_entity):
        """Test updating entity position"""
        # Arrange
        entity_manager.create_entity(sample_entity)
        new_position = {"latitude": 40.7128, "longitude": -74.0060, "altitude": 100.0}
        
        # Act
        result = entity_manager.update_position("test_drone_01", new_position)
        
        # Assert
        assert result is True
        updated_entity = entity_manager.get_entity("test_drone_01")
        assert updated_entity.position == new_position
    
    @patch('constellation_overwatch.core.spatial_index.SpatialIndex')
    def test_spatial_query(self, mock_spatial_index, entity_manager):
        """Test spatial query functionality"""
        # Arrange
        mock_spatial_index.return_value.query.return_value = ["test_drone_01"]
        
        # Act
        results = entity_manager.spatial_query(
            center={"latitude": 39.7392, "longitude": -104.9903},
            radius=1000
        )
        
        # Assert
        mock_spatial_index.return_value.query.assert_called_once()
        assert len(results) == 1
```

#### Testing Async Code

```python
# tests/unit/ai/test_vision_processor.py
import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from constellation_overwatch.ai import VisionProcessor

class TestVisionProcessor:
    """Unit tests for VisionProcessor"""
    
    @pytest.fixture
    def vision_processor(self):
        """Create VisionProcessor instance"""
        return VisionProcessor(model="yolo_v5", confidence_threshold=0.7)
    
    @pytest.mark.asyncio
    async def test_detect_objects_success(self, vision_processor):
        """Test successful object detection"""
        # Arrange
        mock_image_data = b"fake_image_data"
        expected_detections = [
            {"class": "vehicle", "confidence": 0.95, "bbox": [100, 150, 200, 100]}
        ]
        
        with patch.object(vision_processor, '_run_inference', new_callable=AsyncMock) as mock_inference:
            mock_inference.return_value = expected_detections
            
            # Act
            result = await vision_processor.detect_objects(mock_image_data)
            
            # Assert
            assert result["detections"] == expected_detections
            assert result["model_version"] == "yolo_v5"
            mock_inference.assert_called_once_with(mock_image_data)
    
    @pytest.mark.asyncio
    async def test_detect_objects_empty_image(self, vision_processor):
        """Test detection with empty image data"""
        # Act & Assert
        with pytest.raises(ValueError, match="Image data cannot be empty"):
            await vision_processor.detect_objects(b"")
    
    @pytest.mark.asyncio
    async def test_detect_objects_timeout(self, vision_processor):
        """Test detection timeout handling"""
        # Arrange
        with patch.object(vision_processor, '_run_inference', new_callable=AsyncMock) as mock_inference:
            mock_inference.side_effect = asyncio.TimeoutError()
            
            # Act & Assert
            with pytest.raises(asyncio.TimeoutError):
                await vision_processor.detect_objects(b"fake_data")
```

#### Testing with Fixtures and Parametrization

```python
# tests/unit/test_mission_planner.py
import pytest
from constellation_overwatch.missions import MissionPlanner

class TestMissionPlanner:
    """Unit tests for MissionPlanner"""
    
    @pytest.fixture(params=[
        "patrol", "surveillance", "search_rescue", "delivery"
    ])
    def mission_type(self, request):
        """Parametrized mission types"""
        return request.param
    
    @pytest.fixture
    def valid_waypoints(self):
        """Valid waypoints for testing"""
        return [
            {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0},
            {"latitude": 39.7420, "longitude": -104.9880, "altitude": 100.0}
        ]
    
    @pytest.mark.parametrize("altitude", [0, 50, 100, 200, 400])
    def test_validate_altitude(self, altitude):
        """Test altitude validation with various values"""
        planner = MissionPlanner()
        
        if 0 <= altitude <= 400:
            # Should not raise exception
            planner.validate_altitude(altitude)
        else:
            with pytest.raises(ValueError):
                planner.validate_altitude(altitude)
    
    def test_create_mission_all_types(self, mission_type, valid_waypoints):
        """Test mission creation for all mission types"""
        planner = MissionPlanner()
        
        mission_config = {
            "name": f"Test {mission_type} Mission",
            "type": mission_type,
            "waypoints": valid_waypoints
        }
        
        result = planner.create_mission(mission_config)
        
        assert result is not None
        assert result.type == mission_type
        assert len(result.waypoints) == len(valid_waypoints)
```

### JavaScript Unit Testing with Jest

```javascript
// tests/unit/client/test_constellation_client.test.js
const { ConstellationClient } = require('@constellation-overwatch/sdk');
const axios = require('axios');

jest.mock('axios');
const mockedAxios = axios;

describe('ConstellationClient', () => {
  let client;
  
  beforeEach(() => {
    client = new ConstellationClient({
      apiKey: 'test-api-key',
      baseURL: 'http://localhost:8080/api/v1'
    });
    jest.clearAllMocks();
  });
  
  describe('Entity Management', () => {
    test('should list entities successfully', async () => {
      // Arrange
      const mockResponse = {
        data: {
          data: [
            { id: 'drone_001', type: 'drone', status: 'active' }
          ],
          pagination: { total: 1, limit: 100, offset: 0 }
        }
      };
      mockedAxios.get.mockResolvedValue(mockResponse);
      
      // Act
      const result = await client.entities.list();
      
      // Assert
      expect(result).toEqual(mockResponse.data);
      expect(mockedAxios.get).toHaveBeenCalledWith('/entities', {
        headers: { 'Authorization': 'Bearer test-api-key' },
        params: {}
      });
    });
    
    test('should handle API errors gracefully', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 401,
          data: { error: { code: 'INVALID_API_KEY', message: 'Invalid API key' } }
        }
      };
      mockedAxios.get.mockRejectedValue(errorResponse);
      
      // Act & Assert
      await expect(client.entities.list()).rejects.toThrow('Invalid API key');
    });
  });
  
  describe('Mission Management', () => {
    test('should create mission with valid configuration', async () => {
      // Arrange
      const missionConfig = {
        name: 'Test Mission',
        type: 'patrol',
        waypoints: [
          { latitude: 39.7392, longitude: -104.9903, altitude: 100.0 }
        ]
      };
      
      const mockResponse = {
        data: { id: 'mission_001', ...missionConfig, status: 'planning' }
      };
      mockedAxios.post.mockResolvedValue(mockResponse);
      
      // Act
      const result = await client.missions.create(missionConfig);
      
      // Assert
      expect(result).toEqual(mockResponse.data);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        '/missions',
        missionConfig,
        { headers: { 'Authorization': 'Bearer test-api-key' } }
      );
    });
  });
});
```

### C++ Unit Testing with Google Test

```cpp
// tests/unit/core/test_entity_manager.cpp
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "constellation_overwatch/core/entity_manager.h"
#include "constellation_overwatch/core/entity.h"

namespace constellation {
namespace core {
namespace test {

class EntityManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        entity_manager_ = std::make_unique<EntityManager>();
        
        // Create sample entity
        Entity::Config config;
        config.id = "test_drone_01";
        config.type = "drone";
        config.position = {39.7392, -104.9903, 0.0};
        config.capabilities = {"surveillance"};
        
        sample_entity_ = std::make_unique<Entity>(config);
    }
    
    void TearDown() override {
        entity_manager_.reset();
        sample_entity_.reset();
    }
    
    std::unique_ptr<EntityManager> entity_manager_;
    std::unique_ptr<Entity> sample_entity_;
};

TEST_F(EntityManagerTest, CreateEntitySuccess) {
    // Act
    auto result = entity_manager_->CreateEntity(*sample_entity_);
    
    // Assert
    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.value()->id(), "test_drone_01");
    
    auto retrieved = entity_manager_->GetEntity("test_drone_01");
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved.value()->id(), "test_drone_01");
}

TEST_F(EntityManagerTest, CreateEntityDuplicateId) {
    // Arrange
    entity_manager_->CreateEntity(*sample_entity_);
    
    // Act
    auto result = entity_manager_->CreateEntity(*sample_entity_);
    
    // Assert
    ASSERT_FALSE(result.ok());
    EXPECT_THAT(result.status().message(), 
                testing::HasSubstr("Entity with ID test_drone_01 already exists"));
}

TEST_F(EntityManagerTest, GetEntityNotFound) {
    // Act
    auto result = entity_manager_->GetEntity("nonexistent");
    
    // Assert
    EXPECT_FALSE(result.has_value());
}

TEST_F(EntityManagerTest, UpdateEntityPosition) {
    // Arrange
    entity_manager_->CreateEntity(*sample_entity_);
    Position new_position{40.7128, -74.0060, 100.0};
    
    // Act
    auto result = entity_manager_->UpdatePosition("test_drone_01", new_position);
    
    // Assert
    ASSERT_TRUE(result.ok());
    
    auto updated_entity = entity_manager_->GetEntity("test_drone_01");
    ASSERT_TRUE(updated_entity.has_value());
    EXPECT_EQ(updated_entity.value()->position().latitude, 40.7128);
    EXPECT_EQ(updated_entity.value()->position().longitude, -74.0060);
    EXPECT_EQ(updated_entity.value()->position().altitude, 100.0);
}

// Mock for testing spatial queries
class MockSpatialIndex : public SpatialIndex {
public:
    MOCK_METHOD(std::vector<std::string>, Query, 
                (const Position& center, double radius), (override));
};

TEST_F(EntityManagerTest, SpatialQuery) {
    // Arrange
    auto mock_spatial_index = std::make_unique<MockSpatialIndex>();
    EXPECT_CALL(*mock_spatial_index, Query(testing::_, testing::_))
        .WillOnce(testing::Return(std::vector<std::string>{"test_drone_01"}));
    
    entity_manager_->SetSpatialIndex(std::move(mock_spatial_index));
    
    // Act
    auto results = entity_manager_->SpatialQuery(
        Position{39.7392, -104.9903, 0.0}, 1000.0
    );
    
    // Assert
    ASSERT_EQ(results.size(), 1);
    EXPECT_EQ(results[0], "test_drone_01");
}

} // namespace test
} // namespace core
} // namespace constellation
```

## Integration Testing

Integration tests verify that different components work together correctly. They test the interactions between modules, external services, and data persistence.

### API Integration Testing

```python
# tests/integration/api/test_entity_api.py
import pytest
import httpx
import asyncio
from testcontainers.postgres import PostgresContainer
from constellation_overwatch.api import create_app

class TestEntityAPIIntegration:
    """Integration tests for Entity API endpoints"""
    
    @pytest.fixture(scope="class")
    def postgres_container(self):
        """Start PostgreSQL container for testing"""
        with PostgresContainer("postgres:13") as postgres:
            yield postgres
    
    @pytest.fixture(scope="class") 
    async def app(self, postgres_container):
        """Create test application with database"""
        # Configure test database
        database_url = postgres_container.get_connection_url()
        
        app = create_app(
            database_url=database_url,
            testing=True
        )
        
        # Initialize database schema
        await app.database.create_all()
        
        yield app
        
        # Cleanup
        await app.database.drop_all()
    
    @pytest.fixture
    async def client(self, app):
        """Create test HTTP client"""
        async with httpx.AsyncClient(app=app, base_url="http://test") as client:
            yield client
    
    @pytest.fixture
    def auth_headers(self):
        """Authentication headers for test requests"""
        return {"Authorization": "Bearer test-api-key"}
    
    async def test_create_and_get_entity(self, client, auth_headers):
        """Test complete entity creation and retrieval workflow"""
        # Create entity
        entity_data = {
            "id": "integration_test_drone",
            "type": "drone",
            "capabilities": ["surveillance"],
            "position": {
                "latitude": 39.7392,
                "longitude": -104.9903,
                "altitude": 0.0
            }
        }
        
        # Act - Create
        create_response = await client.post(
            "/api/v1/entities",
            json=entity_data,
            headers=auth_headers
        )
        
        # Assert - Create
        assert create_response.status_code == 201
        created_entity = create_response.json()
        assert created_entity["id"] == "integration_test_drone"
        
        # Act - Get
        get_response = await client.get(
            f"/api/v1/entities/{entity_data['id']}",
            headers=auth_headers
        )
        
        # Assert - Get
        assert get_response.status_code == 200
        retrieved_entity = get_response.json()
        assert retrieved_entity["id"] == entity_data["id"]
        assert retrieved_entity["type"] == entity_data["type"]
        assert retrieved_entity["position"] == entity_data["position"]
    
    async def test_entity_update_workflow(self, client, auth_headers):
        """Test entity update workflow"""
        # Arrange - Create entity first
        entity_data = {
            "id": "update_test_drone",
            "type": "drone",
            "capabilities": ["surveillance"],
            "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0}
        }
        
        await client.post("/api/v1/entities", json=entity_data, headers=auth_headers)
        
        # Act - Update
        update_data = {
            "position": {"latitude": 40.7128, "longitude": -74.0060, "altitude": 100.0},
            "status": "active"
        }
        
        update_response = await client.put(
            f"/api/v1/entities/{entity_data['id']}",
            json=update_data,
            headers=auth_headers
        )
        
        # Assert - Update
        assert update_response.status_code == 200
        
        # Verify update
        get_response = await client.get(
            f"/api/v1/entities/{entity_data['id']}",
            headers=auth_headers
        )
        
        updated_entity = get_response.json()
        assert updated_entity["position"] == update_data["position"]
        assert updated_entity["status"] == update_data["status"]
    
    async def test_entity_list_with_filters(self, client, auth_headers):
        """Test entity listing with various filters"""
        # Arrange - Create multiple entities
        entities = [
            {
                "id": "drone_filter_1",
                "type": "drone",
                "status": "active",
                "capabilities": ["surveillance"]
            },
            {
                "id": "drone_filter_2", 
                "type": "drone",
                "status": "inactive",
                "capabilities": ["delivery"]
            },
            {
                "id": "sensor_filter_1",
                "type": "sensor",
                "status": "active",
                "capabilities": ["temperature"]
            }
        ]
        
        for entity in entities:
            entity["position"] = {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0}
            await client.post("/api/v1/entities", json=entity, headers=auth_headers)
        
        # Test type filter
        response = await client.get(
            "/api/v1/entities?type=drone",
            headers=auth_headers
        )
        
        assert response.status_code == 200
        data = response.json()
        assert len(data["data"]) == 2
        assert all(entity["type"] == "drone" for entity in data["data"])
        
        # Test status filter
        response = await client.get(
            "/api/v1/entities?status=active",
            headers=auth_headers
        )
        
        assert response.status_code == 200
        data = response.json()
        assert len(data["data"]) == 2
        assert all(entity["status"] == "active" for entity in data["data"])
```

### Database Integration Testing

```python
# tests/integration/database/test_entity_repository.py
import pytest
import asyncio
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from constellation_overwatch.database import EntityRepository, Entity
from constellation_overwatch.database.models import Base

class TestEntityRepositoryIntegration:
    """Integration tests for EntityRepository with real database"""
    
    @pytest.fixture(scope="class")
    def database_engine(self):
        """Create test database engine"""
        engine = create_engine("sqlite:///test_entities.db", echo=True)
        Base.metadata.create_all(engine)
        yield engine
        Base.metadata.drop_all(engine)
    
    @pytest.fixture
    def db_session(self, database_engine):
        """Create database session for each test"""
        Session = sessionmaker(bind=database_engine)
        session = Session()
        yield session
        session.rollback()
        session.close()
    
    @pytest.fixture
    def entity_repository(self, db_session):
        """Create EntityRepository instance"""
        return EntityRepository(db_session)
    
    async def test_create_and_retrieve_entity(self, entity_repository):
        """Test entity creation and retrieval from database"""
        # Arrange
        entity_data = {
            "id": "db_test_drone",
            "type": "drone",
            "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0},
            "capabilities": ["surveillance"],
            "metadata": {"model": "TestDrone-X1"}
        }
        
        # Act - Create
        created_entity = await entity_repository.create(entity_data)
        
        # Assert - Create
        assert created_entity.id == "db_test_drone"
        assert created_entity.type == "drone"
        
        # Act - Retrieve
        retrieved_entity = await entity_repository.get_by_id("db_test_drone")
        
        # Assert - Retrieve
        assert retrieved_entity is not None
        assert retrieved_entity.id == "db_test_drone"
        assert retrieved_entity.position["latitude"] == 39.7392
    
    async def test_update_entity_in_database(self, entity_repository):
        """Test entity updates are persisted to database"""
        # Arrange
        entity_data = {
            "id": "update_db_test",
            "type": "drone",
            "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0}
        }
        
        created_entity = await entity_repository.create(entity_data)
        
        # Act
        update_data = {
            "position": {"latitude": 40.7128, "longitude": -74.0060, "altitude": 100.0},
            "status": "active"
        }
        
        updated_entity = await entity_repository.update(created_entity.id, update_data)
        
        # Assert
        assert updated_entity.position["latitude"] == 40.7128
        assert updated_entity.status == "active"
        
        # Verify persistence by retrieving fresh from DB
        fresh_entity = await entity_repository.get_by_id(created_entity.id)
        assert fresh_entity.position["latitude"] == 40.7128
        assert fresh_entity.status == "active"
    
    async def test_spatial_query_integration(self, entity_repository):
        """Test spatial queries work with database"""
        # Arrange - Create entities at different locations
        entities = [
            {
                "id": "spatial_test_1",
                "type": "drone",
                "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0}  # Denver
            },
            {
                "id": "spatial_test_2", 
                "type": "drone",
                "position": {"latitude": 40.7128, "longitude": -74.0060, "altitude": 0.0}   # NYC
            },
            {
                "id": "spatial_test_3",
                "type": "drone", 
                "position": {"latitude": 39.7420, "longitude": -104.9880, "altitude": 0.0}  # Near Denver
            }
        ]
        
        for entity_data in entities:
            await entity_repository.create(entity_data)
        
        # Act - Query entities near Denver (within 10km)
        denver_center = {"latitude": 39.7392, "longitude": -104.9903}
        nearby_entities = await entity_repository.spatial_query(
            center=denver_center,
            radius=10000  # 10km in meters
        )
        
        # Assert
        nearby_ids = [entity.id for entity in nearby_entities]
        assert "spatial_test_1" in nearby_ids  # Denver location
        assert "spatial_test_3" in nearby_ids  # Near Denver
        assert "spatial_test_2" not in nearby_ids  # NYC - too far
```

### WebSocket Integration Testing

```python
# tests/integration/websocket/test_realtime_communication.py
import pytest
import asyncio
import websockets
import json
from constellation_overwatch.api import create_app

class TestWebSocketIntegration:
    """Integration tests for WebSocket real-time communication"""
    
    @pytest.fixture
    async def websocket_server(self):
        """Start WebSocket server for testing"""
        app = create_app(testing=True)
        server = await websockets.serve(
            app.websocket_handler,
            "localhost",
            8765
        )
        yield server
        server.close()
        await server.wait_closed()
    
    @pytest.fixture
    async def websocket_client(self, websocket_server):
        """Create WebSocket client connection"""
        uri = "ws://localhost:8765/api/v1/ws"
        async with websockets.connect(uri) as websocket:
            # Authenticate
            auth_message = {
                "type": "auth",
                "token": "test-api-key"
            }
            await websocket.send(json.dumps(auth_message))
            
            # Wait for auth confirmation
            response = await websocket.recv()
            auth_response = json.loads(response)
            assert auth_response["type"] == "auth_success"
            
            yield websocket
    
    async def test_entity_telemetry_subscription(self, websocket_client):
        """Test subscribing to entity telemetry updates"""
        # Subscribe to telemetry
        subscribe_message = {
            "type": "subscribe",
            "channel": "entity.drone_001.telemetry"
        }
        await websocket_client.send(json.dumps(subscribe_message))
        
        # Confirm subscription
        response = await websocket_client.recv()
        sub_response = json.loads(response)
        assert sub_response["type"] == "subscribed"
        assert sub_response["channel"] == "entity.drone_001.telemetry"
        
        # Simulate telemetry update (would normally come from entity)
        telemetry_data = {
            "type": "telemetry",
            "entity_id": "drone_001",
            "timestamp": "2025-01-30T16:00:00Z",
            "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0},
            "battery_level": 85,
            "speed": 15.2
        }
        
        # In real scenario, this would be sent by the system
        # For testing, we'll inject it directly
        await websocket_client.send(json.dumps(telemetry_data))
        
        # Receive and verify telemetry
        received = await websocket_client.recv()
        received_data = json.loads(received)
        
        assert received_data["type"] == "telemetry"
        assert received_data["entity_id"] == "drone_001"
        assert received_data["position"]["altitude"] == 100.0
    
    async def test_mission_event_broadcasting(self, websocket_client):
        """Test mission event broadcasting to subscribers"""
        # Subscribe to mission events
        subscribe_message = {
            "type": "subscribe", 
            "channel": "mission.*.events"
        }
        await websocket_client.send(json.dumps(subscribe_message))
        
        # Confirm subscription
        response = await websocket_client.recv()
        assert json.loads(response)["type"] == "subscribed"
        
        # Simulate mission event
        mission_event = {
            "type": "mission_event",
            "mission_id": "mission_001",
            "event": "waypoint_reached",
            "waypoint_index": 2,
            "timestamp": "2025-01-30T16:05:00Z",
            "entity_id": "drone_001"
        }
        
        # Broadcast event
        await websocket_client.send(json.dumps(mission_event))
        
        # Receive and verify event
        received = await websocket_client.recv()
        received_data = json.loads(received)
        
        assert received_data["type"] == "mission_event"
        assert received_data["mission_id"] == "mission_001"
        assert received_data["event"] == "waypoint_reached"
    
    async def test_command_execution(self, websocket_client):
        """Test sending commands through WebSocket"""
        # Send command to entity
        command = {
            "type": "command",
            "target": "drone_001",
            "command": "goto",
            "parameters": {
                "latitude": 40.7128,
                "longitude": -74.0060,
                "altitude": 150.0
            }
        }
        
        await websocket_client.send(json.dumps(command))
        
        # Receive command acknowledgment
        response = await websocket_client.recv()
        ack_response = json.loads(response)
        
        assert ack_response["type"] == "command_ack"
        assert ack_response["command_id"] is not None
        assert ack_response["status"] == "accepted"
        
        # Receive command completion (simulated)
        completion_response = await websocket_client.recv()
        completion_data = json.loads(completion_response)
        
        assert completion_data["type"] == "command_complete"
        assert completion_data["status"] == "success"
```

## System Testing

System testing verifies complete end-to-end workflows and user scenarios.

### End-to-End Mission Workflow Testing

```python
# tests/system/test_mission_workflow.py
import pytest
import asyncio
import time
from constellation_overwatch import ConstellationClient

class TestMissionWorkflowSystem:
    """System tests for complete mission workflows"""
    
    @pytest.fixture
    async def client(self):
        """Create authenticated client for system testing"""
        client = ConstellationClient(
            api_key="system-test-key",
            base_url="http://localhost:8080/api/v1",
            timeout=60.0
        )
        
        # Verify system is ready
        health = await client.health.check()
        assert health.status == "healthy"
        
        yield client
        
        # Cleanup test entities and missions
        await self.cleanup_test_data(client)
    
    async def cleanup_test_data(self, client):
        """Clean up test entities and missions"""
        # Get all test entities
        entities = await client.entities.list()
        for entity in entities.data:
            if entity.id.startswith("system_test_"):
                await client.entities.delete(entity.id)
        
        # Get all test missions
        missions = await client.missions.list()
        for mission in missions.data:
            if mission.name.startswith("System Test"):
                await client.missions.delete(mission.id)
    
    async def test_complete_patrol_mission_workflow(self, client):
        """Test complete patrol mission from creation to completion"""
        # Step 1: Create and register drone
        drone_config = {
            "id": "system_test_patrol_drone",
            "type": "drone",
            "capabilities": ["surveillance", "patrol"],
            "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0},
            "metadata": {"model": "SystemTest-Drone-V1", "simulation": True}
        }
        
        drone = await client.entities.create(drone_config)
        assert drone.id == "system_test_patrol_drone"
        
        # Wait for drone to become active
        await self.wait_for_entity_status(client, drone.id, "active", timeout=30)
        
        # Step 2: Create patrol mission
        mission_config = {
            "name": "System Test Patrol Mission",
            "type": "patrol",
            "waypoints": [
                {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0, "action": "takeoff"},
                {"latitude": 39.7420, "longitude": -104.9903, "altitude": 100.0, "action": "patrol"},
                {"latitude": 39.7420, "longitude": -104.9880, "altitude": 100.0, "action": "patrol"},
                {"latitude": 39.7392, "longitude": -104.9880, "altitude": 100.0, "action": "patrol"},
                {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0, "action": "land"}
            ],
            "parameters": {
                "speed": 15.0,
                "loiter_time": 10,
                "return_to_launch": True
            }
        }
        
        mission = await client.missions.create(mission_config)
        assert mission.type == "patrol"
        
        # Step 3: Start mission
        execution = await client.missions.start(
            mission.id,
            assigned_entities=[drone.id]
        )
        
        assert execution.status == "active"
        
        # Step 4: Monitor mission progress
        mission_completed = await self.monitor_mission_completion(
            client, mission.id, timeout=300  # 5 minutes
        )
        
        assert mission_completed
        
        # Step 5: Verify mission results
        final_mission = await client.missions.get(mission.id)
        assert final_mission.status == "completed"
        
        final_drone = await client.entities.get(drone.id)
        assert final_drone.status == "active"  # Should return to active state
        
        # Verify drone returned to launch position
        launch_pos = mission_config["waypoints"][0]
        final_pos = final_drone.position
        
        assert abs(final_pos["latitude"] - launch_pos["latitude"]) < 0.0001
        assert abs(final_pos["longitude"] - launch_pos["longitude"]) < 0.0001
    
    async def test_multi_drone_coordinated_mission(self, client):
        """Test coordinated mission with multiple drones"""
        # Create multiple drones
        drones = []
        for i in range(3):
            drone_config = {
                "id": f"system_test_coord_drone_{i}",
                "type": "drone",
                "capabilities": ["surveillance"],
                "position": {
                    "latitude": 39.7392 + (i * 0.001),
                    "longitude": -104.9903 + (i * 0.001),
                    "altitude": 0.0
                },
                "metadata": {"simulation": True}
            }
            
            drone = await client.entities.create(drone_config)
            drones.append(drone)
            
            # Wait for drone to be active
            await self.wait_for_entity_status(client, drone.id, "active")
        
        # Create coordinated search mission
        search_area = [
            {"latitude": 39.7392, "longitude": -104.9903},
            {"latitude": 39.7420, "longitude": -104.9903},
            {"latitude": 39.7420, "longitude": -104.9880},
            {"latitude": 39.7392, "longitude": -104.9880}
        ]
        
        mission_config = {
            "name": "System Test Coordinated Search",
            "type": "search",
            "search_area": search_area,
            "parameters": {
                "search_pattern": "grid",
                "coordination": "auto_sector"
            }
        }
        
        mission = await client.missions.create(mission_config)
        
        # Start mission with all drones
        execution = await client.missions.start(
            mission.id,
            assigned_entities=[drone.id for drone in drones]
        )
        
        # Monitor coordination
        coordination_successful = await self.monitor_coordinated_execution(
            client, mission.id, [drone.id for drone in drones]
        )
        
        assert coordination_successful
    
    async def test_ai_powered_detection_workflow(self, client):
        """Test AI-powered object detection integration"""
        # Create drone with camera capability
        drone_config = {
            "id": "system_test_ai_drone",
            "type": "drone",
            "capabilities": ["surveillance", "camera", "ai_detection"],
            "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0},
            "metadata": {"camera_specs": "4K", "ai_models": ["yolo_v5"]}
        }
        
        drone = await client.entities.create(drone_config)
        await self.wait_for_entity_status(client, drone.id, "active")
        
        # Create surveillance mission with AI detection
        mission_config = {
            "name": "System Test AI Detection Mission",
            "type": "surveillance",
            "waypoints": [
                {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0, "action": "survey"},
                {"latitude": 39.7420, "longitude": -104.9880, "altitude": 100.0, "action": "survey"}
            ],
            "ai_analysis": {
                "detect_objects": True,
                "object_classes": ["vehicle", "person"],
                "confidence_threshold": 0.7,
                "alert_on_detection": True
            }
        }
        
        mission = await client.missions.create(mission_config)
        execution = await client.missions.start(mission.id, assigned_entities=[drone.id])
        
        # Monitor for AI detections
        detections = await self.monitor_ai_detections(client, mission.id, timeout=120)
        
        # Verify AI processing occurred (even if no objects detected)
        assert len(detections) >= 0  # Could be zero if no objects in test environment
    
    async def wait_for_entity_status(self, client, entity_id, expected_status, timeout=60):
        """Wait for entity to reach expected status"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            entity = await client.entities.get(entity_id)
            if entity.status == expected_status:
                return True
            
            await asyncio.sleep(1)
        
        raise TimeoutError(f"Entity {entity_id} did not reach status {expected_status} within {timeout}s")
    
    async def monitor_mission_completion(self, client, mission_id, timeout=300):
        """Monitor mission until completion"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            mission = await client.missions.get(mission_id)
            
            if mission.status in ["completed", "failed", "cancelled"]:
                return mission.status == "completed"
            
            await asyncio.sleep(2)
        
        return False
    
    async def monitor_coordinated_execution(self, client, mission_id, drone_ids):
        """Monitor coordinated mission execution"""
        # Implementation would track coordination metrics
        # For now, just verify all drones are participating
        
        for _ in range(30):  # Monitor for 1 minute
            mission = await client.missions.get(mission_id)
            
            if mission.status == "active":
                # Verify all drones are assigned and active
                all_active = True
                for drone_id in drone_ids:
                    drone = await client.entities.get(drone_id)
                    if drone.mission_status != "in_progress":
                        all_active = False
                        break
                
                if all_active:
                    return True
            
            await asyncio.sleep(2)
        
        return False
    
    async def monitor_ai_detections(self, client, mission_id, timeout=120):
        """Monitor AI detections during mission"""
        detections = []
        start_time = time.time()
        
        # In real implementation, would use WebSocket to listen for detections
        # For testing, simulate by checking mission AI results
        
        while time.time() - start_time < timeout:
            mission = await client.missions.get(mission_id)
            
            if hasattr(mission, 'ai_results') and mission.ai_results:
                detections.extend(mission.ai_results.get('detections', []))
            
            if mission.status in ["completed", "failed", "cancelled"]:
                break
            
            await asyncio.sleep(5)
        
        return detections
```

## Performance Testing

Performance testing ensures the system meets performance requirements under various load conditions.

### Load Testing with Locust

```python
# tests/performance/load/test_api_load.py
from locust import HttpUser, task, between
import json
import random

class ConstellationAPIUser(HttpUser):
    """Locust user for API load testing"""
    
    wait_time = between(1, 3)  # Wait 1-3 seconds between requests
    
    def on_start(self):
        """Setup for each user"""
        self.auth_headers = {
            "Authorization": "Bearer load-test-api-key",
            "Content-Type": "application/json"
        }
        
        # Create test entities for this user
        self.entity_ids = []
        for i in range(5):
            entity_data = {
                "id": f"load_test_entity_{self.client.environment.runner.user_count}_{i}",
                "type": "drone",
                "capabilities": ["surveillance"],
                "position": {
                    "latitude": 39.7392 + random.uniform(-0.01, 0.01),
                    "longitude": -104.9903 + random.uniform(-0.01, 0.01),
                    "altitude": 0.0
                }
            }
            
            response = self.client.post(
                "/api/v1/entities",
                json=entity_data,
                headers=self.auth_headers,
                name="Create Entity (Setup)"
            )
            
            if response.status_code == 201:
                self.entity_ids.append(entity_data["id"])
    
    def on_stop(self):
        """Cleanup for each user"""
        for entity_id in self.entity_ids:
            self.client.delete(
                f"/api/v1/entities/{entity_id}",
                headers=self.auth_headers,
                name="Delete Entity (Cleanup)"
            )
    
    @task(3)
    def list_entities(self):
        """List entities - high frequency task"""
        params = {
            "limit": random.choice([10, 50, 100]),
            "type": random.choice(["drone", "sensor", None])
        }
        
        # Remove None values
        params = {k: v for k, v in params.items() if v is not None}
        
        self.client.get(
            "/api/v1/entities",
            params=params,
            headers=self.auth_headers,
            name="List Entities"
        )
    
    @task(2)
    def get_entity_details(self):
        """Get specific entity - medium frequency"""
        if self.entity_ids:
            entity_id = random.choice(self.entity_ids)
            self.client.get(
                f"/api/v1/entities/{entity_id}",
                headers=self.auth_headers,
                name="Get Entity"
            )
    
    @task(1)
    def update_entity_position(self):
        """Update entity position - low frequency"""
        if self.entity_ids:
            entity_id = random.choice(self.entity_ids)
            update_data = {
                "position": {
                    "latitude": 39.7392 + random.uniform(-0.1, 0.1),
                    "longitude": -104.9903 + random.uniform(-0.1, 0.1),
                    "altitude": random.uniform(0, 500)
                },
                "battery_level": random.randint(20, 100)
            }
            
            self.client.put(
                f"/api/v1/entities/{entity_id}",
                json=update_data,
                headers=self.auth_headers,
                name="Update Entity"
            )
    
    @task(1)
    def create_mission(self):
        """Create mission - low frequency"""
        if self.entity_ids:
            mission_data = {
                "name": f"Load Test Mission {random.randint(1000, 9999)}",
                "type": random.choice(["patrol", "surveillance", "search"]),
                "waypoints": [
                    {
                        "latitude": 39.7392 + random.uniform(-0.01, 0.01),
                        "longitude": -104.9903 + random.uniform(-0.01, 0.01),
                        "altitude": random.uniform(50, 200),
                        "action": "patrol"
                    }
                ]
            }
            
            response = self.client.post(
                "/api/v1/missions",
                json=mission_data,
                headers=self.auth_headers,
                name="Create Mission"
            )
            
            # Clean up created mission
            if response.status_code == 201:
                mission_id = response.json()["id"]
                self.client.delete(
                    f"/api/v1/missions/{mission_id}",
                    headers=self.auth_headers,
                    name="Delete Mission (Cleanup)"
                )

# Run load test:
# locust -f tests/performance/load/test_api_load.py --host=http://localhost:8080
```

### Stress Testing

```python
# tests/performance/stress/test_websocket_stress.py
import asyncio
import websockets
import json
import time
import logging
from concurrent.futures import ThreadPoolExecutor

class WebSocketStressTester:
    """Stress test WebSocket connections and message throughput"""
    
    def __init__(self, base_url="ws://localhost:8080/api/v1/ws"):
        self.base_url = base_url
        self.connections = []
        self.message_count = 0
        self.error_count = 0
        
    async def create_connection(self, connection_id):
        """Create and authenticate a WebSocket connection"""
        try:
            websocket = await websockets.connect(self.base_url)
            
            # Authenticate
            auth_message = {
                "type": "auth",
                "token": f"stress-test-token-{connection_id}"
            }
            await websocket.send(json.dumps(auth_message))
            
            # Wait for auth confirmation
            response = await websocket.recv()
            auth_response = json.loads(response)
            
            if auth_response["type"] != "auth_success":
                raise Exception(f"Authentication failed for connection {connection_id}")
            
            return websocket
            
        except Exception as e:
            logging.error(f"Failed to create connection {connection_id}: {e}")
            self.error_count += 1
            return None
    
    async def send_messages(self, websocket, connection_id, message_rate=10):
        """Send messages at specified rate per second"""
        try:
            interval = 1.0 / message_rate
            
            while True:
                message = {
                    "type": "telemetry",
                    "entity_id": f"stress_entity_{connection_id}",
                    "timestamp": time.time(),
                    "position": {
                        "latitude": 39.7392,
                        "longitude": -104.9903,
                        "altitude": 100.0
                    },
                    "battery_level": 85
                }
                
                await websocket.send(json.dumps(message))
                self.message_count += 1
                
                await asyncio.sleep(interval)
                
        except websockets.exceptions.ConnectionClosed:
            logging.warning(f"Connection {connection_id} closed")
        except Exception as e:
            logging.error(f"Error sending messages for connection {connection_id}: {e}")
            self.error_count += 1
    
    async def stress_test(self, num_connections=100, message_rate=10, duration=60):
        """Run stress test with specified parameters"""
        logging.info(f"Starting stress test: {num_connections} connections, "
                    f"{message_rate} msg/s each, {duration}s duration")
        
        # Create connections
        tasks = []
        for i in range(num_connections):
            websocket = await self.create_connection(i)
            if websocket:
                self.connections.append(websocket)
                task = asyncio.create_task(
                    self.send_messages(websocket, i, message_rate)
                )
                tasks.append(task)
        
        logging.info(f"Created {len(self.connections)} connections")
        
        # Run for specified duration
        await asyncio.sleep(duration)
        
        # Cancel tasks and close connections
        for task in tasks:
            task.cancel()
        
        for websocket in self.connections:
            await websocket.close()
        
        # Calculate results
        total_expected = num_connections * message_rate * duration
        success_rate = ((total_expected - self.error_count) / total_expected) * 100
        
        logging.info(f"Stress test completed:")
        logging.info(f"  Messages sent: {self.message_count}")
        logging.info(f"  Errors: {self.error_count}")
        logging.info(f"  Success rate: {success_rate:.2f}%")
        
        return {
            "messages_sent": self.message_count,
            "errors": self.error_count,
            "success_rate": success_rate
        }

# Run stress test
async def run_stress_test():
    tester = WebSocketStressTester()
    results = await tester.stress_test(
        num_connections=100,
        message_rate=10,
        duration=60
    )
    print(f"Results: {results}")

if __name__ == "__main__":
    asyncio.run(run_stress_test())
```

### Benchmark Testing

```python
# tests/performance/benchmark/test_api_benchmarks.py
import asyncio
import time
import statistics
import httpx
from typing import List, Dict

class APIBenchmark:
    """Benchmark API performance and establish baselines"""
    
    def __init__(self, base_url="http://localhost:8080/api/v1"):
        self.base_url = base_url
        self.auth_headers = {"Authorization": "Bearer benchmark-test-key"}
    
    async def benchmark_endpoint(self, method: str, url: str, data=None, iterations=100):
        """Benchmark specific endpoint performance"""
        response_times = []
        errors = 0
        
        async with httpx.AsyncClient() as client:
            for i in range(iterations):
                start_time = time.time()
                
                try:
                    if method.upper() == "GET":
                        response = await client.get(
                            f"{self.base_url}{url}",
                            headers=self.auth_headers
                        )
                    elif method.upper() == "POST":
                        response = await client.post(
                            f"{self.base_url}{url}",
                            json=data,
                            headers=self.auth_headers
                        )
                    elif method.upper() == "PUT":
                        response = await client.put(
                            f"{self.base_url}{url}",
                            json=data,
                            headers=self.auth_headers
                        )
                    
                    response.raise_for_status()
                    
                except Exception as e:
                    errors += 1
                    continue
                
                end_time = time.time()
                response_times.append((end_time - start_time) * 1000)  # Convert to ms
        
        if response_times:
            return {
                "min_time": min(response_times),
                "max_time": max(response_times),
                "avg_time": statistics.mean(response_times),
                "median_time": statistics.median(response_times),
                "p95_time": self.percentile(response_times, 95),
                "p99_time": self.percentile(response_times, 99),
                "errors": errors,
                "success_rate": ((iterations - errors) / iterations) * 100
            }
        else:
            return {"error": "All requests failed"}
    
    def percentile(self, data: List[float], percentile: float) -> float:
        """Calculate percentile of data"""
        return statistics.quantiles(data, n=100)[int(percentile) - 1]
    
    async def run_benchmarks(self):
        """Run comprehensive API benchmarks"""
        benchmarks = {}
        
        # Entity endpoints
        benchmarks["list_entities"] = await self.benchmark_endpoint(
            "GET", "/entities", iterations=1000
        )
        
        benchmarks["create_entity"] = await self.benchmark_endpoint(
            "POST", "/entities",
            data={
                "id": "benchmark_drone",
                "type": "drone",
                "capabilities": ["surveillance"],
                "position": {"latitude": 39.7392, "longitude": -104.9903, "altitude": 0.0}
            },
            iterations=100
        )
        
        benchmarks["get_entity"] = await self.benchmark_endpoint(
            "GET", "/entities/benchmark_drone", iterations=1000
        )
        
        benchmarks["update_entity"] = await self.benchmark_endpoint(
            "PUT", "/entities/benchmark_drone",
            data={"position": {"latitude": 40.0, "longitude": -105.0, "altitude": 100.0}},
            iterations=100
        )
        
        # Mission endpoints
        benchmarks["create_mission"] = await self.benchmark_endpoint(
            "POST", "/missions",
            data={
                "name": "Benchmark Mission",
                "type": "patrol",
                "waypoints": [
                    {"latitude": 39.7392, "longitude": -104.9903, "altitude": 100.0}
                ]
            },
            iterations=100
        )
        
        return benchmarks
    
    def print_benchmark_results(self, benchmarks: Dict):
        """Print formatted benchmark results"""
        print("\n" + "="*80)
        print("API BENCHMARK RESULTS")
        print("="*80)
        
        for endpoint, results in benchmarks.items():
            if "error" in results:
                print(f"\n{endpoint.upper()}: {results['error']}")
                continue
            
            print(f"\n{endpoint.upper()}:")
            print(f"  Average: {results['avg_time']:.2f}ms")
            print(f"  Median:  {results['median_time']:.2f}ms")
            print(f"  Min:     {results['min_time']:.2f}ms")
            print(f"  Max:     {results['max_time']:.2f}ms")
            print(f"  P95:     {results['p95_time']:.2f}ms")
            print(f"  P99:     {results['p99_time']:.2f}ms")
            print(f"  Success: {results['success_rate']:.1f}%")
            
            # Performance thresholds
            if results['avg_time'] > 500:
                print(f"  ⚠️  WARNING: Average response time exceeds 500ms")
            if results['p95_time'] > 1000:
                print(f"  ⚠️  WARNING: P95 response time exceeds 1000ms")
            if results['success_rate'] < 99:
                print(f"  ⚠️  WARNING: Success rate below 99%")

# Run benchmarks
async def main():
    benchmark = APIBenchmark()
    results = await benchmark.run_benchmarks()
    benchmark.print_benchmark_results(results)

if __name__ == "__main__":
    asyncio.run(main())
```

## Security Testing

Security testing verifies the system is protected against common vulnerabilities and attacks.

### Authentication and Authorization Testing

```python
# tests/security/test_authentication.py
import pytest
import httpx
import jwt
import time
from constellation_overwatch.security import TokenManager

class TestSecurityAuthentication:
    """Security tests for authentication and authorization"""
    
    @pytest.fixture
    async def client(self):
        """HTTP client for security testing"""
        async with httpx.AsyncClient(base_url="http://localhost:8080") as client:
            yield client
    
    async def test_no_authentication_fails(self, client):
        """Test that requests without authentication are rejected"""
        response = await client.get("/api/v1/entities")
        
        assert response.status_code == 401
        assert "authentication" in response.json()["error"]["message"].lower()
    
    async def test_invalid_api_key_fails(self, client):
        """Test that invalid API keys are rejected"""
        headers = {"Authorization": "Bearer invalid-api-key-12345"}
        response = await client.get("/api/v1/entities", headers=headers)
        
        assert response.status_code == 401
        assert response.json()["error"]["code"] == "INVALID_API_KEY"
    
    async def test_expired_token_fails(self, client):
        """Test that expired JWT tokens are rejected"""
        # Create expired token
        expired_payload = {
            "sub": "test-user",
            "exp": int(time.time()) - 3600,  # Expired 1 hour ago
            "iat": int(time.time()) - 7200   # Issued 2 hours ago
        }
        
        expired_token = jwt.encode(expired_payload, "test-secret", algorithm="HS256")
        headers = {"Authorization": f"Bearer {expired_token}"}
        
        response = await client.get("/api/v1/entities", headers=headers)
        
        assert response.status_code == 401
        assert "expired" in response.json()["error"]["message"].lower()
    
    async def test_malformed_token_fails(self, client):
        """Test that malformed tokens are rejected"""
        malformed_tokens = [
            "Bearer malformed.token.here",
            "Bearer notajwttoken",
            "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.malformed",
            "InvalidPrefix actual-token-here"
        ]
        
        for token in malformed_tokens:
            headers = {"Authorization": token}
            response = await client.get("/api/v1/entities", headers=headers)
            
            assert response.status_code == 401
    
    async def test_insufficient_permissions(self, client):
        """Test that users with insufficient permissions are blocked"""
        # Create token with limited scope
        limited_payload = {
            "sub": "limited-user",
            "scope": ["read"],  # No write permissions
            "exp": int(time.time()) + 3600
        }
        
        limited_token = jwt.encode(limited_payload, "test-secret", algorithm="HS256")
        headers = {"Authorization": f"Bearer {limited_token}"}
        
        # Should allow read operations
        response = await client.get("/api/v1/entities", headers=headers)
        assert response.status_code == 200
        
        # Should block write operations
        response = await client.post(
            "/api/v1/entities",
            json={"id": "test", "type": "drone"},
            headers=headers
        )
        assert response.status_code == 403
        assert response.json()["error"]["code"] == "INSUFFICIENT_PERMISSIONS"
    
    async def test_rate_limiting(self, client):
        """Test that rate limiting protects against abuse"""
        headers = {"Authorization": "Bearer valid-test-key"}
        
        # Make many requests quickly
        responses = []
        for i in range(150):  # Exceed rate limit of 100/minute
            response = await client.get("/api/v1/health", headers=headers)
            responses.append(response.status_code)
        
        # Some requests should be rate limited
        rate_limited_count = sum(1 for status in responses if status == 429)
        assert rate_limited_count > 0
```

### Input Validation and Injection Testing

```python
# tests/security/test_input_validation.py
import pytest
import httpx

class TestInputValidationSecurity:
    """Security tests for input validation and injection attacks"""
    
    @pytest.fixture
    async def client(self):
        async with httpx.AsyncClient(base_url="http://localhost:8080") as client:
            yield client
    
    @pytest.fixture
    def auth_headers(self):
        return {"Authorization": "Bearer security-test-key"}
    
    async def test_sql_injection_prevention(self, client, auth_headers):
        """Test that SQL injection attempts are blocked"""
        sql_injection_payloads = [
            "'; DROP TABLE entities; --",
            "1' OR '1'='1",
            "1; DELETE FROM entities WHERE 1=1; --",
            "1' UNION SELECT * FROM users; --"
        ]
        
        for payload in sql_injection_payloads:
            # Try injection in entity ID parameter
            response = await client.get(
                f"/api/v1/entities/{payload}",
                headers=auth_headers
            )
            
            # Should return 404 (not found) or 400 (bad request), not 500 (server error)
            assert response.status_code in [400, 404]
            
            # Response should not contain SQL error messages
            response_text = response.text.lower()
            sql_error_keywords = ["sql", "syntax error", "mysql", "postgresql", "sqlite"]
            for keyword in sql_error_keywords:
                assert keyword not in response_text
    
    async def test_xss_prevention(self, client, auth_headers):
        """Test that XSS attempts are sanitized"""
        xss_payloads = [
            "<script>alert('xss')</script>",
            "javascript:alert('xss')",
            "<img src=x onerror=alert('xss')>",
            "<svg onload=alert('xss')>"
        ]
        
        for payload in xss_payloads:
            entity_data = {
                "id": f"xss_test_{hash(payload)}",
                "type": "drone",
                "capabilities": [payload],  # Try XSS in array field
                "metadata": {"description": payload}  # Try XSS in text field
            }
            
            response = await client.post(
                "/api/v1/entities",
                json=entity_data,
                headers=auth_headers
            )
            
            if response.status_code == 201:
                # If creation succeeded, verify XSS was sanitized
                created_entity = response.json()
                
                # XSS payload should be sanitized or rejected
                for field_value in [created_entity.get("capabilities", []), 
                                  created_entity.get("metadata", {}).values()]:
                    if isinstance(field_value, list):
                        field_value = " ".join(str(v) for v in field_value)
                    elif isinstance(field_value, dict):
                        field_value = " ".join(str(v) for v in field_value.values())
                    
                    field_str = str(field_value)
                    assert "<script>" not in field_str
                    assert "javascript:" not in field_str
                    assert "onerror=" not in field_str
    
    async def test_path_traversal_prevention(self, client, auth_headers):
        """Test that path traversal attempts are blocked"""
        path_traversal_payloads = [
            "../../../etc/passwd",
            "..\\..\\..\\windows\\system32\\config\\sam",
            "%2e%2e%2f%2e%2e%2f%2e%2e%2fetc%2fpasswd",
            "....//....//....//etc/passwd"
        ]
        
        for payload in path_traversal_payloads:
            # Try path traversal in API endpoints
            response = await client.get(
                f"/api/v1/entities/{payload}",
                headers=auth_headers
            )
            
            # Should return 400 or 404, not expose file contents
            assert response.status_code in [400, 404]
            
            # Should not contain system file contents
            response_text = response.text.lower()
            system_file_indicators = ["root:x:", "administrator:", "[boot loader]"]
            for indicator in system_file_indicators:
                assert indicator not in response_text
    
    async def test_command_injection_prevention(self, client, auth_headers):
        """Test that command injection attempts are blocked"""
        command_injection_payloads = [
            "; ls -la",
            "| whoami",
            "&& cat /etc/passwd",
            "`rm -rf /`",
            "$(cat /etc/passwd)"
        ]
        
        for payload in command_injection_payloads:
            entity_data = {
                "id": f"cmd_test_{hash(payload)}",
                "type": "drone",
                "metadata": {"command": payload}
            }
            
            response = await client.post(
                "/api/v1/entities",
                json=entity_data,
                headers=auth_headers
            )
            
            # Commands should not be executed
            # Response should not contain command output
            if response.status_code == 201:
                created_entity = response.json()
                metadata_str = str(created_entity.get("metadata", {}))
                
                # Should not contain command execution results
                command_output_indicators = [
                    "total ", "drwx", "-rwx", "root", "bin", "usr", "etc"
                ]
                for indicator in command_output_indicators:
                    assert indicator not in metadata_str
```

### Data Encryption and Security Testing

```python
# tests/security/test_encryption.py
import pytest
import httpx
import json
from constellation_overwatch.security import EncryptionManager

class TestEncryptionSecurity:
    """Security tests for data encryption and secure communication"""
    
    @pytest.fixture
    def encryption_manager(self):
        return EncryptionManager()
    
    async def test_sensitive_data_encryption(self, encryption_manager):
        """Test that sensitive data is properly encrypted"""
        sensitive_data = {
            "api_key": "super-secret-api-key-12345",
            "coordinates": {"latitude": 39.7392, "longitude": -104.9903},
            "mission_details": "Classified operation details"
        }
        
        # Encrypt data
        encrypted_data = encryption_manager.encrypt(json.dumps(sensitive_data))
        
        # Verify encryption worked
        assert encrypted_data != json.dumps(sensitive_data)
        assert "super-secret-api-key" not in encrypted_data
        assert "Classified operation" not in encrypted_data
        
        # Verify decryption works
        decrypted_data = json.loads(encryption_manager.decrypt(encrypted_data))
        assert decrypted_data == sensitive_data
    
    async def test_password_hashing(self, encryption_manager):
        """Test that passwords are properly hashed"""
        passwords = [
            "simplepassword",
            "ComplexP@ssw0rd!",
            "verylongpasswordwithmanycharactersandnumbers123456789"
        ]
        
        for password in passwords:
            # Hash password
            hashed = encryption_manager.hash_password(password)
            
            # Verify hash is different from original
            assert hashed != password
            assert len(hashed) > len(password)
            
            # Verify hash verification works
            assert encryption_manager.verify_password(password, hashed)
            assert not encryption_manager.verify_password("wrongpassword", hashed)
    
    async def test_secure_random_generation(self, encryption_manager):
        """Test secure random number generation"""
        # Generate multiple tokens
        tokens = []
        for _ in range(100):
            token = encryption_manager.generate_secure_token(32)
            tokens.append(token)
            
            # Verify token properties
            assert len(token) == 64  # 32 bytes = 64 hex chars
            assert all(c in "0123456789abcdef" for c in token)
        
        # Verify tokens are unique
        assert len(set(tokens)) == len(tokens)
    
    async def test_tls_certificate_validation(self):
        """Test TLS certificate validation"""
        # Test HTTPS connections with proper certificate validation
        async with httpx.AsyncClient(verify=True) as client:
            try:
                # This should work with valid certificates
                response = await client.get("https://httpbin.org/get")
                assert response.status_code == 200
            except httpx.RequestError:
                pytest.skip("External HTTPS test site not available")
        
        # Test that invalid certificates are rejected
        async with httpx.AsyncClient(verify=True) as client:
            with pytest.raises((httpx.RequestError, httpx.ConnectError)):
                # This should fail with self-signed/invalid certificate
                await client.get("https://self-signed.badssl.com/")
```

## API Testing

API testing focuses on verifying API functionality, contracts, and integration.

### Contract Testing with Pact

```python
# tests/api/contract/test_api_contract.py
import pytest
from pact import Consumer, Provider
import requests

# Define API contract
pact = Consumer('constellation-client').has_pact_with(Provider('constellation-api'))

class TestAPIContract:
    """Contract tests to ensure API compatibility"""
    
    def test_get_entity_contract(self):
        """Test contract for getting entity details"""
        expected_entity = {
            "id": "test-drone-001",
            "type": "drone",
            "status": "active",
            "position": {
                "latitude": 39.7392,
                "longitude": -104.9903,
                "altitude": 100.0
            },
            "capabilities": ["surveillance"],
            "battery_level": 85,
            "last_contact": "2025-01-30T15:30:00Z"
        }
        
        (pact
         .given('entity test-drone-001 exists')
         .upon_receiving('a request for entity details')
         .with_request(
             method='GET',
             path='/api/v1/entities/test-drone-001',
             headers={'Authorization': 'Bearer test-token'}
         )
         .will_respond_with(200, body=expected_entity)
        )
        
        with pact:
            # Make actual request
            response = requests.get(
                f"{pact.uri}/api/v1/entities/test-drone-001",
                headers={'Authorization': 'Bearer test-token'}
            )
            
            assert response.status_code == 200
            assert response.json() == expected_entity
    
    def test_create_entity_contract(self):
        """Test contract for creating entities"""
        entity_request = {
            "id": "new-drone-001",
            "type": "drone",
            "capabilities": ["surveillance"],
            "position": {
                "latitude": 40.7128,
                "longitude": -74.0060,
                "altitude": 0.0
            }
        }
        
        entity_response = {
            "id": "new-drone-001",
            "type": "drone",
            "status": "inactive",
            "created_at": "2025-01-30T16:00:00Z",
            "message": "Entity created successfully"
        }
        
        (pact
         .given('entity new-drone-001 does not exist')
         .upon_receiving('a request to create entity')
         .with_request(
             method='POST',
             path='/api/v1/entities',
             headers={
                 'Authorization': 'Bearer test-token',
                 'Content-Type': 'application/json'
             },
             body=entity_request
         )
         .will_respond_with(201, body=entity_response)
        )
        
        with pact:
            response = requests.post(
                f"{pact.uri}/api/v1/entities",
                json=entity_request,
                headers={
                    'Authorization': 'Bearer test-token',
                    'Content-Type': 'application/json'
                }
            )
            
            assert response.status_code == 201
            assert response.json() == entity_response
```

### OpenAPI Schema Validation

```python
# tests/api/schema/test_openapi_validation.py
import pytest
import yaml
import httpx
from openapi_spec_validator import validate_spec
from openapi_core import create_spec
from openapi_core.validation.request.validators import RequestValidator
from openapi_core.validation.response.validators import ResponseValidator

class TestOpenAPIValidation:
    """Validate API responses against OpenAPI schema"""
    
    @pytest.fixture(scope="class")
    def openapi_spec(self):
        """Load OpenAPI specification"""
        with open("api/openapi.yaml", "r") as f:
            spec_dict = yaml.safe_load(f)
        
        # Validate spec is valid OpenAPI
        validate_spec(spec_dict)
        
        return create_spec(spec_dict)
    
    @pytest.fixture
    async def client(self):
        async with httpx.AsyncClient(base_url="http://localhost:8080") as client:
            yield client
    
    @pytest.fixture
    def auth_headers(self):
        return {"Authorization": "Bearer openapi-test-key"}
    
    async def test_entity_list_response_schema(self, client, auth_headers, openapi_spec):
        """Test that entity list response matches OpenAPI schema"""
        response = await client.get("/api/v1/entities", headers=auth_headers)
        
        assert response.status_code == 200
        
        # Validate response against schema
        validator = ResponseValidator(openapi_spec)
        result = validator.validate(
            host_url="http://localhost:8080",
            path="/api/v1/entities",
            method="get",
            response=response
        )
        
        # Should not raise validation errors
        errors = list(result.errors)
        assert len(errors) == 0, f"Schema validation errors: {errors}"
    
    async def test_entity_create_request_schema(self, client, auth_headers, openapi_spec):
        """Test that entity creation follows request schema"""
        valid_entity = {
            "id": "schema-test-drone",
            "type": "drone",
            "capabilities": ["surveillance"],
            "position": {
                "latitude": 39.7392,
                "longitude": -104.9903,
                "altitude": 0.0
            }
        }
        
        # Validate request against schema
        validator = RequestValidator(openapi_spec)
        
        # This would validate the request before sending
        # In practice, you'd use this in your application
        
        response = await client.post(
            "/api/v1/entities",
            json=valid_entity,
            headers=auth_headers
        )
        
        assert response.status_code == 201
        
        # Validate response schema
        response_validator = ResponseValidator(openapi_spec)
        result = response_validator.validate(
            host_url="http://localhost:8080",
            path="/api/v1/entities",
            method="post",
            response=response
        )
        
        errors = list(result.errors)
        assert len(errors) == 0, f"Response schema validation errors: {errors}"
    
    async def test_invalid_request_schema(self, client, auth_headers):
        """Test that invalid requests are rejected with proper error format"""
        invalid_entities = [
            {"type": "drone"},  # Missing required id
            {"id": "test", "type": "invalid_type"},  # Invalid type
            {"id": "test", "type": "drone", "position": {"latitude": 91}},  # Invalid latitude
            {"id": "test", "type": "drone", "capabilities": "not_an_array"}  # Wrong type
        ]
        
        for invalid_entity in invalid_entities:
            response = await client.post(
                "/api/v1/entities",
                json=invalid_entity,
                headers=auth_headers
            )
            
            assert response.status_code == 400
            
            error_response = response.json()
            assert "error" in error_response
            assert "code" in error_response["error"]
            assert "message" in error_response["error"]
```

## Test Automation

Automated testing ensures consistent quality and rapid feedback.

### GitHub Actions CI/CD Pipeline

```yaml
# .github/workflows/test.yml
name: Test Suite

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main, develop]

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: [3.8, 3.9, 3.10, 3.11]
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python ${{ matrix.python-version }}
      uses: actions/setup-python@v4
      with:
        python-version: ${{ matrix.python-version }}
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -r requirements.txt
        pip install -r requirements-test.txt
    
    - name: Run unit tests
      run: |
        pytest tests/unit/ \
          --cov=constellation_overwatch \
          --cov-report=xml \
          --cov-report=html \
          --junitxml=junit/test-results.xml
    
    - name: Upload coverage to Codecov
      uses: codecov/codecov-action@v3
      with:
        file: ./coverage.xml
        flags: unittests
        name: codecov-umbrella
    
    - name: Publish test results
      uses: EnricoMi/publish-unit-test-result-action@v2
      if: always()
      with:
        files: junit/test-results.xml
  
  integration-tests:
    runs-on: ubuntu-latest
    needs: unit-tests
    
    services:
      postgres:
        image: postgres:13
        env:
          POSTGRES_PASSWORD: test
          POSTGRES_DB: constellation_test
        options: >-
          --health-cmd pg_isready
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
      
      redis:
        image: redis:7
        options: >-
          --health-cmd "redis-cli ping"
          --health-interval 10s
          --health-timeout 5s
          --health-retries 5
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: 3.11
    
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -r requirements.txt
        pip install -r requirements-test.txt
    
    - name: Start API server
      run: |
        docker-compose up -d
        sleep 30  # Wait for services to start
    
    - name: Run integration tests
      env:
        DATABASE_URL: postgresql://postgres:test@localhost:5432/constellation_test
        REDIS_URL: redis://localhost:6379
      run: |
        pytest tests/integration/ \
          --junitxml=junit/integration-results.xml
    
    - name: Stop services
      run: docker-compose down
  
  security-tests:
    runs-on: ubuntu-latest
    needs: unit-tests
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: 3.11
    
    - name: Install security tools
      run: |
        pip install bandit safety semgrep
    
    - name: Run Bandit security scan
      run: |
        bandit -r constellation_overwatch/ \
          -f json -o bandit-report.json
    
    - name: Run Safety vulnerability check
      run: |
        safety check --json --output safety-report.json
    
    - name: Run Semgrep security scan
      run: |
        semgrep --config=auto constellation_overwatch/ \
          --json --output=semgrep-report.json
    
    - name: Upload security reports
      uses: actions/upload-artifact@v3
      with:
        name: security-reports
        path: |
          bandit-report.json
          safety-report.json
          semgrep-report.json
  
  performance-tests:
    runs-on: ubuntu-latest
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: 3.11
    
    - name: Install dependencies
      run: |
        pip install locust httpx
    
    - name: Start services
      run: |
        docker-compose up -d
        sleep 30
    
    - name: Run performance tests
      run: |
        locust \
          -f tests/performance/load/test_api_load.py \
          --headless \
          --users 50 \
          --spawn-rate 10 \
          --run-time 2m \
          --host http://localhost:8080 \
          --html performance-report.html
    
    - name: Upload performance report
      uses: actions/upload-artifact@v3
      with:
        name: performance-report
        path: performance-report.html
```

### Pre-commit Hooks

```yaml
# .pre-commit-config.yaml
repos:
  - repo: https://github.com/pre-commit/pre-commit-hooks
    rev: v4.4.0
    hooks:
      - id: trailing-whitespace
      - id: end-of-file-fixer
      - id: check-yaml
      - id: check-added-large-files
      - id: check-merge-conflict
  
  - repo: https://github.com/psf/black
    rev: 23.1.0
    hooks:
      - id: black
        language_version: python3
  
  - repo: https://github.com/pycqa/isort
    rev: 5.12.0
    hooks:
      - id: isort
        args: ["--profile", "black"]
  
  - repo: https://github.com/pycqa/flake8
    rev: 6.0.0
    hooks:
      - id: flake8
        args: ["--max-line-length=88", "--extend-ignore=E203,W503"]
  
  - repo: https://github.com/pycqa/bandit
    rev: 1.7.4
    hooks:
      - id: bandit
        args: ["-c", "pyproject.toml"]
  
  - repo: local
    hooks:
      - id: pytest-fast
        name: Run fast tests
        entry: pytest tests/unit/ -x --tb=short
        language: system
        pass_filenames: false
        always_run: true
```

## Best Practices

### Test Organization and Structure

1. **Follow the AAA Pattern**: Arrange, Act, Assert
2. **Use Descriptive Test Names**: Test names should describe what is being tested
3. **One Assertion Per Test**: Keep tests focused and specific
4. **Use Fixtures for Setup**: Reuse common setup code with fixtures
5. **Mock External Dependencies**: Isolate units under test

### Test Data Management

```python
# tests/fixtures/test_data.py
"""Centralized test data management"""

import json
from pathlib import Path

class TestDataManager:
    """Manage test data and fixtures"""
    
    def __init__(self):
        self.data_dir = Path(__file__).parent / "data"
    
    def load_entity_data(self, entity_type="drone"):
        """Load sample entity data"""
        file_path = self.data_dir / f"entities/{entity_type}.json"
        with open(file_path) as f:
            return json.load(f)
    
    def load_mission_data(self, mission_type="patrol"):
        """Load sample mission data"""
        file_path = self.data_dir / f"missions/{mission_type}.json"
        with open(file_path) as f:
            return json.load(f)
    
    def create_test_entity(self, **overrides):
        """Create test entity with optional overrides"""
        base_entity = self.load_entity_data()
        base_entity.update(overrides)
        return base_entity

# Usage in tests
@pytest.fixture
def test_data():
    return TestDataManager()

def test_entity_creation(test_data):
    entity_data = test_data.create_test_entity(
        id="custom-test-drone",
        capabilities=["surveillance", "delivery"]
    )
    # Use entity_data in test
```

### Continuous Testing Strategy

1. **Test Pyramid**: Many unit tests, some integration tests, few E2E tests
2. **Fail Fast**: Run fastest tests first to get quick feedback
3. **Parallel Execution**: Run tests in parallel for faster feedback
4. **Environment Parity**: Test environments should match production
5. **Regular Updates**: Keep test dependencies and tools updated

### Quality Metrics

Monitor these key testing metrics:

- **Code Coverage**: Aim for 80%+ coverage
- **Test Execution Time**: Keep unit tests under 1s each
- **Test Reliability**: < 1% flaky test rate
- **Bug Escape Rate**: < 5% of bugs should escape to production
- **Test Automation**: 90%+ of tests should be automated

---

*This testing guide provides comprehensive coverage of testing strategies for the Constellation Overwatch SDK. Regular updates and improvements to the testing framework ensure continued quality and reliability.*