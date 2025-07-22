"""
Constellation Overwatch SDK - Integrated Demo Server
Combines REST API, WebSocket, and serves web dashboard.
"""

import asyncio
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
import json
import time
from typing import List
import os

from sdk.core.entity_manager import EntityManager, create_drone_entity, create_ground_station_entity
from sdk.core.message_bus import MessageBus, MessageType, publish_entity_event
from sdk.api.rest_server import ConstellationAPI


class IntegratedConstellationServer:
    """
    Integrated server combining REST API, WebSocket, and web dashboard.
    Demonstrates the functional core with full web interface.
    """
    
    def __init__(self):
        self.api = ConstellationAPI()
        self.setup_static_files()
        
    def setup_static_files(self):
        """Setup static file serving for web dashboard"""
        
        # Add static file serving for web assets
        web_dir = os.path.join(os.path.dirname(__file__), "..", "..", "web")
        if os.path.exists(web_dir):
            self.api.app.mount("/static", StaticFiles(directory=web_dir), name="static")
        
        # Add dashboard route
        @self.api.app.get("/dashboard", response_class=HTMLResponse)
        async def dashboard():
            """Serve the web dashboard"""
            dashboard_path = os.path.join(
                os.path.dirname(__file__), "..", "..", "web", "dashboard.html"
            )
            if os.path.exists(dashboard_path):
                with open(dashboard_path, 'r', encoding='utf-8') as f:
                    return HTMLResponse(content=f.read())
            else:
                return HTMLResponse(
                    content="<h1>Dashboard not found</h1><p>Please ensure web/dashboard.html exists</p>",
                    status_code=404
                )
        
        # Redirect root to dashboard
        @self.api.app.get("/", response_class=HTMLResponse)
        async def root_redirect():
            return HTMLResponse(
                content="""
                <html>
                <head>
                    <meta http-equiv="refresh" content="0; url=/dashboard">
                    <title>Constellation Overwatch</title>
                    <style>
                        body { 
                            font-family: Arial, sans-serif; 
                            background: #1a1a2e; 
                            color: white; 
                            text-align: center; 
                            padding: 2rem;
                        }
                        .loading { color: #00ff88; }
                    </style>
                </head>
                <body>
                    <h1>🛰️ Constellation Overwatch</h1>
                    <p class="loading">Redirecting to dashboard...</p>
                    <p><a href="/dashboard" style="color: #00ff88;">Click here if not redirected automatically</a></p>
                </body>
                </html>
                """
            )
    
    async def start_with_demo_data(self):
        """Start server with some demo data"""
        await self.api.start()
        
        # Create demo entities
        print("Creating demo entities...")
        
        # Ground Control Station
        gcs = create_ground_station_entity(40.7505, -73.9934, "GCS-Main")
        await self.api.entity_manager.publish_entity(gcs)
        
        # Demo drones
        drone1 = create_drone_entity(40.7128, -74.0060, 0, "Alpha-01")
        drone2 = create_drone_entity(40.7589, -73.9851, 0, "Beta-02")
        
        await self.api.entity_manager.publish_entity(drone1)
        await self.api.entity_manager.publish_entity(drone2)
        
        # Start vehicle interfaces for demo
        import sys
        import os
        sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        from examples.functional_core.demo import SimpleVehicleInterface
        
        vehicle1 = SimpleVehicleInterface(drone1.entity_id, self.api.message_bus)
        vehicle2 = SimpleVehicleInterface(drone2.entity_id, self.api.message_bus)
        
        await vehicle1.start()
        await vehicle2.start()
        
        print(f"Demo server started with:")
        print(f"  - {len([gcs, drone1, drone2])} entities")
        print(f"  - 2 vehicle interfaces")
        print(f"  - REST API at http://localhost:8000")
        print(f"  - Web Dashboard at http://localhost:8000/dashboard")
        print(f"  - WebSocket at ws://localhost:8000/ws")
        
        return vehicle1, vehicle2
    
    async def stop(self):
        """Stop the integrated server"""
        await self.api.stop()


async def run_demo_mission(api: ConstellationAPI, vehicles: List):
    """Run an automated demo mission"""
    print("\n🚀 Starting automated demo mission...")
    
    # Wait a bit for everything to initialize
    await asyncio.sleep(2)
    
    # Get vehicle IDs
    vehicle_ids = [v.vehicle_id for v in vehicles]
    
    # Mission sequence
    print("Phase 1: Takeoff sequence")
    for i, vehicle_id in enumerate(vehicle_ids):
        altitude = 50 + (i * 25)  # Staggered altitudes
        await api.message_bus.publish(
            message_type=MessageType.VEHICLE_COMMAND,
            source="demo_mission",
            target=vehicle_id,
            topic="commands",
            payload={"command": "takeoff", "altitude": altitude},
            priority=5
        )
        await asyncio.sleep(2)
    
    await asyncio.sleep(5)
    
    print("Phase 2: Navigation sequence")
    waypoints = [
        {"lat": 40.7200, "lon": -74.0100},
        {"lat": 40.7600, "lon": -73.9800}
    ]
    
    for i, vehicle_id in enumerate(vehicle_ids):
        if i < len(waypoints):
            await api.message_bus.publish(
                message_type=MessageType.VEHICLE_COMMAND,
                source="demo_mission",
                target=vehicle_id,
                topic="commands",
                payload={"command": "goto", "waypoint": waypoints[i]},
                priority=5
            )
        await asyncio.sleep(1)
    
    await asyncio.sleep(8)
    
    print("Phase 3: Landing sequence")
    for vehicle_id in vehicle_ids:
        await api.message_bus.publish(
            message_type=MessageType.VEHICLE_COMMAND,
            source="demo_mission",
            target=vehicle_id,
            topic="commands",
            payload={"command": "land"},
            priority=5
        )
        await asyncio.sleep(2)
    
    print("✅ Demo mission complete!")


async def main():
    """Main server entry point"""
    print("🛰️ Starting Constellation Overwatch Integrated Server")
    print("=" * 60)
    
    server = IntegratedConstellationServer()
    
    try:
        # Start server with demo data
        vehicles = await server.start_with_demo_data()
        
        # Start automated demo mission in background
        asyncio.create_task(run_demo_mission(server.api, vehicles))
        
        # Start the web server
        config = uvicorn.Config(
            app=server.api.app,
            host="0.0.0.0",
            port=8000,
            log_level="info"
        )
        
        server_instance = uvicorn.Server(config)
        await server_instance.serve()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        await server.stop()


if __name__ == "__main__":
    asyncio.run(main())
