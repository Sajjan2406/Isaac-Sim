# Isaac Sim + iPhysics Integration via JSON (OPC UA Prep)

This project demonstrates how to connect **iPhysics** and **Isaac Sim** using a simulated OPC UA interface via JSON. 
It reads positional data from a JSON file and updates a model's position in Isaac Sim in real-time.
![Actuator in IsaacSim](images\Actuator.png)
## Simulation Video
The simulation video can be watcherd here:[![Watch the Simulation Video] (https://youtu.be/st5fMZlsggA)]
## 🚀 Features

- Real-time model manipulation in Isaac Sim
- JSON-based simulation of OPC UA data
- Non-blocking update loop using Isaac Sim's event stream
- Easy to extend to OPC UA client

## 📁 Project Structure
- `scripts/update_position.py`: Main script to read JSON and update model
- `data/Translation.json`: Sample data file with position and rotation
- `docs/architecture.md`: System overview and future OPC UA roadmap
- `USD/Actuator.usd`: Actuator usd file used in the project

## 🛠 Requirements

- Isaac Sim 4.5+
- Python (use Isaac Sim's built-in interpreter)

## 📈 Next Steps

- Replace JSON with OPC UA client using `opcua` Python library
- Add rotation support
- Build a UI panel in Isaac Sim for manual control
