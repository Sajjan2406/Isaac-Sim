import time
import asyncio
from asyncua import Client
from pxr import Gf
import omni.usd
import omni.kit.app

# Prim path
prim_path = "/Actuator/Actuator/tn__3DSMesh_3_"

# Shared value between threads
latest_value = {"z": 0.0}

last_update_time = [0]

#Update Positional value in the isaacsim Environment
def update_position(event):
    print("Update position called")
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        print(f"Prim not found: {prim_path}")
        return

    # Get the attribute to manipulate
    attr = prim.GetAttribute("xformOp:translate")
    if not attr or not attr.IsValid():
        print("Translate attribute not found.")
        return

    # set the values for the attribute
    now = time.time()
    if now - last_update_time[0] < 0.02:
        return

    try:
        x, y, z = 0.0, 0.0, -1*latest_value["z"]
        attr.Set(Gf.Vec3f(x, y, z))
        last_update_time[0] = now
        print(f"Applied translation: x={x:.2f}, y={y:.2f}, z={z:.2f}")
    except Exception as e:
        print(f"Error applying translation: {e}")


#OPC UA Client
async def opcua_reader():
    url = "opc.tcp://vibn-id523:4840/"
    node_id = "ns=1;s=_8048309_EGSC-BS-10000027_0.PositionRead"

    async with Client(url=url) as client:
        print("Connected to OPC UA server")
        node = client.get_node(node_id)

        while True:
            try:
                value = await node.get_value()
                latest_value["z"] = float(value)
                print(f"Read OPC UA value: {value}")
            except Exception as e:
                print(f"Error reading OPC UA value: {e}")
            await asyncio.sleep(0.1)


update_stream = None
# Register update callback
def register_update():
    global update_stream
    update_stream = omni.kit.app.get_app().get_update_event_stream()
    update_stream.create_subscription_to_pop(update_position)
    print("Update function registered")

register_update()

# Start OPC UA reader in background
asyncio.ensure_future(opcua_reader())

