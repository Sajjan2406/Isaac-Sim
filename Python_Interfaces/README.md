


# Isaac Sim Python Interface to simulate real-time communication

This project demonstrates how to use Python extensions within Isaac Sim to simulate real-time communication with external softwares. Positional data is read from a JSON file and used to update the actuator model's position inside Isaac Sim. This setup serves as a prototype for future OPC UA integration.

![Actuator in IsaacSim](images\Actuator.png)
Watch the simulation of model manipulation in Isaac Sim using Python here:![Watch the Simulation Video] https://youtu.be/st5fMZlsggA

## 📁 Project Structure
- `scripts/update_position.py`: Python script to read JSON and update model position
- `data/Translation.json`: Sample data file containing position and rotation values
- `docs/architecture.md`: System overview and roadmap for OPC UA integration
- `USD/Actuator.usd`: Actuator USD file used in the simulation

## 🛠 Requirements
- Isaac Sim 4.5+
- Python (use Isaac Sim's built-in interpreter)

## 📈 Next Steps
- Replace JSON input with live OPC UA client using the `opcua` Python library
- Add support for rotation and velocity updates
- Build a UI panel in Isaac Sim for manual control and debugging

---

This project was developed during my internship at **Krones**, focusing on Python-based simulation interfaces and preparing for industrial OPC UA communication workflows.
