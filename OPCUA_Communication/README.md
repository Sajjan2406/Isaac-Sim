# Isaac Sim + iPhysics Integration via OPC UA

This project demonstrates the communication between **iPhysics** and **Isaac Sim** using a simulated OPC UA interface. The goal is to synchronize the digital twin of an actuator in Isaac Sim with positional data from iPhysics.

## Simulation
Watch the actuator's digital twin being manipulated via positional data from iPhysics [here](https://youtu.be/9UtRx_iPYpY)

## ⚙️ Technologies Used
- **Isaac Sim**
- **iPhysics**
- **Python**
- **OPC UA Protocol**

## 📁 Project Structure
- `scripts/OPC_UA_Client.py`: Main OPC UA Client Script
- `3d_models`: Contains the 3d Models of the Actuator

## Description
- **iPhysics** acts as the **OPC UA server**, publishing actuator position data via the control tag `PositionRead`.
![alttext](<images/iPhysics.png>)

- **Isaac Sim** acts as the **OPC UA client**, reading this data and updating the actuator's position in real-time, using a Python script built with the Isaac Sim API

![alttext](<images/IsaacSim.png>)





