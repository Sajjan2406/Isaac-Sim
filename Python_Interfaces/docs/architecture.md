# System Architecture

## Overview

This integration simulates OPC UA communication using a JSON file as a mock data source. The goal is to eventually replace this with a live OPC UA server for real-time industrial communication. The project bridges iPhysics and Isaac Sim, enabling synchronized simulation of mechanical systems and robotic environments.

Importantly, Isaac Sim is not used solely for visualization - it hosts a functional digital twin of an actuator, modeled with realistic kinematics using prismatic joints, fixed joints, and other constraints. This allows for behavior simulation, not just rendering.
## Components

**iPhysics**: 
- Simulates mechanical systems (e.g., conveyors, bottles, robots).
- Generates real-time positional data.
- Acts as the OPC UA server
- Currently outputs data to a JSON file (mock OPC UA Server).

**JSON File**: 
- Acts as a temporary substitute for an OPC UA server.
- Stores object positions, rotations, and timestamps.
- Enables decoupled communication between iPhysics and Isaac Sim.

**Isaac Sim**: 
- Hosts a working digital twin of an actuator, modeled with:
  - Prismatic joints for linear motion.
  - Fixed joints for rigid connections.
  - Kinematic chains to simulate realistic mechanical behavior.
  - Receives data from the Python bridge and updates the scene.
        
**Python Script**: 
- Reads and parses the JSON file.
- Converts data into Isaac Sim-compatible formats.
- Updates object states in Isaac Sim via its Python API.

## Data Flow

![](https://github.com/Sajjan2406/Isaac-Sim/blob/4ad68805b04090a7670b38b8736e6bf77ee8794f/Python_Interfaces/images/WorkFlow.png)
## Future Roadmap

- Integrate real OPC UA server
- Add bidirectional communication
- Sync with iPhysics simulation timeline
