#!/usr/bin/env python3
"""
Constellation Overwatch - Quick Start Launcher
Launches the integrated demo server with functional core and web dashboard.
"""

import sys
import os
import asyncio

# Add the project root to Python path
project_root = os.path.dirname(__file__)
sys.path.insert(0, project_root)

try:
    # Fix import path by using correct module name
    from examples.integrated_demo.server import main
    
    print("🛰️  Constellation Overwatch - Functional Core Demo")
    print("=" * 60)
    print("Starting integrated server with:")
    print("  ✅ Entity Management System")
    print("  ✅ Message Bus Communication") 
    print("  ✅ Vehicle Interface Simulation")
    print("  ✅ REST API (http://localhost:8000)")
    print("  ✅ Real-time WebSocket")
    print("  ✅ Web Dashboard (http://localhost:8000/dashboard)")
    print()
    print("This demonstrates our FUNCTIONAL CORE FIRST approach:")
    print("  • Working implementation validates architecture")
    print("  • Real data flows inform API design") 
    print("  • Immediate demonstrable value")
    print("=" * 60)
    print()
    
    # Run the server
    asyncio.run(main())
    
except ImportError as e:
    print(f"❌ Import Error: {e}")
    print()
    print("Missing dependencies. Please install:")
    print("  python -m pip install fastapi uvicorn websockets")
    print()
    sys.exit(1)
    
except KeyboardInterrupt:
    print("\n👋 Shutdown complete. Thanks for trying Constellation Overwatch!")
    
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
