import time
import json
from pxr import Gf
import omni.usd
import omni.kit.app

# Prim and JSON file path
prim_path = "/Actuator/Actuator/tn__3DSMesh_3_"
json_file = r"C:\isaacsim_projects\extensions\First_Extension\Translation.json"

# Timer variables
last_update_time = [0]

def update_position(event):
    
    #specify the stage
    stage = omni.usd.get_context().get_stage()
    
    #specify the prim to be manipiulated
    prim = stage.GetPrimAtPath(prim_path)

    if not prim.IsValid():
        print(f"Prim not found: {prim_path}")
        return
    
    #Get the attribute of the prim to be manipulated
    attr = prim.GetAttribute("xformOp:translate")

    if not attr or not attr.IsValid():
        print(" translate attribute not found.")
        return
    
    now = time.time()
    if now - last_update_time[0] < 0.02: #interval to update the position value
        return
    try:
        with open(json_file, "r") as f:
            data = json.load(f)

        pos = data.get("position", {})
        x = float(pos.get("x", 0.0))
        y = float(pos.get("y", 0.0))
        z = float(pos.get("z", 0.0))

        #set the tranlational values 
        attr.Set(Gf.Vec3f(x, y, z))

        last_update_time[0] = now
        print(f" Applied translation: x={x:.2f}, y={y:.2f}, z={z:.2f}")
    except Exception as e:
        print(f" Error reading or applying translation: {e}")

# Register the update function
update_stream = omni.kit.app.get_app().get_update_event_stream()
# subscription
update_stream.create_subscription_to_pop(update_position)

# unsubscription 
#update_stream = None 

