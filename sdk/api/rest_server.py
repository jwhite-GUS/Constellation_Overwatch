"""
Constellation Overwatch SDK - REST API Server
FastAPI wrapper around the functional core components.
"""

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import asyncio
import json
import time
from typing import Dict, List, Any, Optional
from pydantic import BaseModel
import uvicorn

from sdk.core.entity_manager import EntityManager, EntityType, create_drone_entity, create_ground_station_entity
from sdk.core.message_bus import MessageBus, MessageType, Message


# Pydantic models for API requests/responses
class EntityResponse(BaseModel):
    entity_id: str
    entity_type: str
    timestamp: float
    is_live: bool
    position: Optional[Dict[str, Any]] = None
    components: Dict[str, Any] = {}
    aliases: Dict[str, str] = {}

class VehicleCommand(BaseModel):
    command: str
    parameters: Dict[str, Any] = {}

class CreateEntityRequest(BaseModel):
    entity_type: str
    position: Optional[Dict[str, float]] = None
    name: Optional[str] = None

class SystemStats(BaseModel):
    entity_manager: Dict[str, Any]
    message_bus: Dict[str, Any]
    uptime: float


class ConstellationAPI:
    """
    REST API server wrapping the functional core.
    Provides HTTP endpoints for the working entity management and messaging system.
    """
    
    def __init__(self):
        self.app = FastAPI(
            title="Constellation Overwatch API",
            description="Government autonomy platform - functional core API",
            version="0.1.0"
        )
        self.entity_manager = EntityManager()
        self.message_bus = MessageBus()
        self.websocket_connections: List[WebSocket] = []
        self.start_time = time.time()
        
        self._setup_routes()
        self._setup_websocket()
    
    def _setup_routes(self):
        """Setup REST API routes"""
        
        @self.app.get("/")
        async def root():
            return {
                "message": "Constellation Overwatch API",
                "status": "operational",
                "uptime": time.time() - self.start_time
            }
        
        @self.app.get("/health")
        async def health_check():
            return {"status": "healthy", "timestamp": time.time()}
        
        # Entity Management Endpoints
        @self.app.get("/entities", response_model=List[EntityResponse])
        async def get_entities(entity_type: Optional[str] = None, active_only: bool = True):
            """Get all entities with optional filtering"""
            try:
                filter_type = EntityType(entity_type) if entity_type else None
                entities = await self.entity_manager.query_entities(filter_type, active_only)
                
                return [
                    EntityResponse(
                        entity_id=e.entity_id,
                        entity_type=e.entity_type.value,
                        timestamp=e.timestamp,
                        is_live=e.is_live,
                        position=e.position.__dict__ if e.position else None,
                        components={k: v.data for k, v in e.components.items()},
                        aliases=e.aliases
                    )
                    for e in entities
                ]
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.get("/entities/{entity_id}", response_model=EntityResponse)
        async def get_entity(entity_id: str):
            """Get specific entity by ID"""
            entity = await self.entity_manager.get_entity(entity_id)
            if not entity:
                raise HTTPException(status_code=404, detail="Entity not found")
            
            return EntityResponse(
                entity_id=entity.entity_id,
                entity_type=entity.entity_type.value,
                timestamp=entity.timestamp,
                is_live=entity.is_live,
                position=entity.position.__dict__ if entity.position else None,
                components={k: v.data for k, v in entity.components.items()},
                aliases=entity.aliases
            )
        
        @self.app.post("/entities", response_model=EntityResponse)
        async def create_entity(request: CreateEntityRequest):
            """Create a new entity"""
            try:
                if request.entity_type == "aircraft_multirotor" and request.position:
                    entity = create_drone_entity(
                        lat=request.position.get("latitude", 0),
                        lon=request.position.get("longitude", 0),
                        alt=request.position.get("altitude", 0),
                        name=request.name
                    )
                elif request.entity_type == "operator_station" and request.position:
                    entity = create_ground_station_entity(
                        lat=request.position.get("latitude", 0),
                        lon=request.position.get("longitude", 0),
                        name=request.name
                    )
                else:
                    raise HTTPException(status_code=400, detail="Unsupported entity type or missing position")
                
                success = await self.entity_manager.publish_entity(entity)
                if not success:
                    raise HTTPException(status_code=500, detail="Failed to create entity")
                
                return EntityResponse(
                    entity_id=entity.entity_id,
                    entity_type=entity.entity_type.value,
                    timestamp=entity.timestamp,
                    is_live=entity.is_live,
                    position=entity.position.__dict__ if entity.position else None,
                    components={k: v.data for k, v in entity.components.items()},
                    aliases=entity.aliases
                )
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.delete("/entities/{entity_id}")
        async def delete_entity(entity_id: str):
            """Delete an entity"""
            success = await self.entity_manager.remove_entity(entity_id)
            if not success:
                raise HTTPException(status_code=404, detail="Entity not found")
            return {"message": "Entity deleted successfully"}
        
        # Vehicle Command Endpoints
        @self.app.post("/vehicles/{vehicle_id}/commands")
        async def send_vehicle_command(vehicle_id: str, command: VehicleCommand):
            """Send command to specific vehicle"""
            try:
                # Verify vehicle exists
                entity = await self.entity_manager.get_entity(vehicle_id)
                if not entity:
                    raise HTTPException(status_code=404, detail="Vehicle not found")
                
                # Send command via message bus
                message_id = await self.message_bus.publish(
                    message_type=MessageType.VEHICLE_COMMAND,
                    source="rest_api",
                    target=vehicle_id,
                    topic="commands",
                    payload={
                        "command": command.command,
                        **command.parameters
                    },
                    priority=5
                )
                
                return {
                    "message": "Command sent successfully",
                    "message_id": message_id,
                    "vehicle_id": vehicle_id,
                    "command": command.command
                }
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        # System Status Endpoints
        @self.app.get("/system/stats", response_model=SystemStats)
        async def get_system_stats():
            """Get system statistics"""
            return SystemStats(
                entity_manager=self.entity_manager.get_stats(),
                message_bus=self.message_bus.get_stats(),
                uptime=time.time() - self.start_time
            )
        
        @self.app.get("/system/messages")
        async def get_recent_messages(limit: int = 50):
            """Get recent message statistics"""
            return self.message_bus.get_stats()
    
    def _setup_websocket(self):
        """Setup WebSocket for real-time updates"""
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.websocket_connections.append(websocket)
            
            try:
                # Subscribe to message bus for real-time updates
                async def websocket_message_handler(message: Message):
                    try:
                        await websocket.send_text(json.dumps({
                            "type": "message",
                            "data": message.to_dict()
                        }))
                    except:
                        pass  # Connection closed
                
                await self.message_bus.subscribe(websocket_message_handler)
                
                # Keep connection alive
                while True:
                    try:
                        data = await websocket.receive_text()
                        # Echo back for heartbeat
                        await websocket.send_text(json.dumps({
                            "type": "pong",
                            "timestamp": time.time()
                        }))
                    except WebSocketDisconnect:
                        break
            
            except WebSocketDisconnect:
                pass
            finally:
                if websocket in self.websocket_connections:
                    self.websocket_connections.remove(websocket)
    
    async def start(self):
        """Start the API server and core components"""
        await self.entity_manager.start()
        await self.message_bus.start()
        
        # Connect entity manager to message bus for real-time updates
        async def entity_event_publisher(event_type: str, entity):
            from sdk.core.message_bus import publish_entity_event
            await publish_entity_event(
                self.message_bus,
                event_type,
                entity.entity_id,
                entity.to_dict()
            )
        
        await self.entity_manager.subscribe(entity_event_publisher)
        print("Constellation Overwatch API started")
    
    async def stop(self):
        """Stop the API server and core components"""
        await self.entity_manager.stop()
        await self.message_bus.stop()
        print("Constellation Overwatch API stopped")


# Standalone server runner
async def create_api_server() -> ConstellationAPI:
    """Create and start the API server"""
    api = ConstellationAPI()
    await api.start()
    return api


def run_server(host: str = "0.0.0.0", port: int = 8000):
    """Run the REST API server"""
    
    async def startup():
        api = await create_api_server()
        return api.app
    
    # For development - in production would use proper ASGI server
    uvicorn.run(startup, host=host, port=port, factory=True)


if __name__ == "__main__":
    print("Starting Constellation Overwatch REST API Server...")
    run_server()
