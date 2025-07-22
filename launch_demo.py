#!/usr/bin/env python3
"""
Constellation Overwatch - Quick Start Launcher
Launches the integrated demo server with functional core and web dashboard.

COPILOT: Professional launcher script following established formatting standards
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
    
    print("CONSTELLATION OVERWATCH - Functional Core Demo")
    print("=" * 60)
    print("Starting integrated server with:")
    print("  COMPLETE: Entity Management System")
    print("  COMPLETE: Message Bus Communication") 
    print("  COMPLETE: Vehicle Interface Simulation")
    print("  READY: REST API (http://localhost:8000)")
    print("  READY: Real-time WebSocket")
    print("  READY: Web Dashboard (http://localhost:8000/dashboard)")
    print()
    print("This demonstrates our FUNCTIONAL CORE FIRST approach:")
    print("  - Working implementation validates architecture")
    print("  - Real data flows inform API design") 
    print("  - Immediate demonstrable value")
    print("=" * 60)
    print()
    
    # Run the server
    asyncio.run(main())
    
except ImportError as e:
    print(f"IMPORT ERROR: {e}")
    print()
    print("Missing dependencies. Please install:")
    print("  python -m pip install fastapi uvicorn websockets")
    print()
    sys.exit(1)
    
except KeyboardInterrupt:
    print("\nSHUTDOWN COMPLETE: Thanks for trying Constellation Overwatch!")
    
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)
