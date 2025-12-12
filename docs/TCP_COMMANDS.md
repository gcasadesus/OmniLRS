# TCP Command Reference

Complete reference for all available TCP commands in OmniLRS.

## Connection

```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5555))
```

---

## Robot Management Commands

### Robot Velocity Control
```json
{
  "/robot_name": {
    "cmd": "vel",
    "val": [linear_x, angular_z]
  }
}
```
- `linear_x`: Forward/backward velocity (m/s)
- `angular_z`: Rotation velocity (rad/s)
- **Return:** `{"success": true}` (Does NOT return full state)
- **Note:** Commands persist until changed. Send `[0.0, 0.0]` to stop.

### Get Robot State
```json
{
  "cmd": "get_state"
}
```
- Returns full state dictionary for all robots (position, orientation, velocity).


### Teleport Robot
```json
{
  "cmd": "teleport",
  "robot_name": "/husky",
  "position": [x, y, z],
  "orientation": [w, x, y, z]
}
```

### Spawn Robot
```json
{
  "cmd": "spawn_robot",
  "robot_name": "/new_robot",
  "usd_path": "assets/USD_Assets/robots/ros2_husky_PhysX_vlp16.usd",
  "position": [x, y, z],
  "orientation": [w, x, y, z]
}
```

### Reset Robot
```json
{
  "cmd": "reset_robot",
  "robot_name": "/husky"
}
```

### Reset All Robots
```json
{
  "cmd": "reset_all_robots"
}
```

### Toggle Wheel Physics
```json
{
  "cmd": "wheel_mode",
  "val": true
}
```
- `true`: Use wheel-based physics (wheels rotate)
- `false`: Use root driving (no wheel rotation, but more reliable)
- **Default:** `true`

---

---

## Camera Commands

### Get Camera Image (RGB)
```json
{
  "cmd": "get_camera_image",
  "robot_name": "/husky",
  "camera": "left",
  "resolution": "high"
}
```
- Returns base64 encoded PNG image.

### Get Camera Depth
```json
{
  "cmd": "get_camera_depth",
  "robot_name": "/husky",
  "camera": "left",
  "resolution": "high"
}
```
- Returns base64 encoded raw float32 bytes (numpy compatible).

---

## Projector Commands

### Turn On/Off
```json
{
  "cmd": "projector",
  "action": "turn_on",
  "value": true
}
```

### Set Intensity
```json
{
  "cmd": "projector",
  "action": "set_intensity",
  "value": 75.0
}
```
- Value: 0-100 (percentage)

### Set Radius
```json
{
  "cmd": "projector",
  "action": "set_radius",
  "value": 2.5
}
```
- Value: radius in meters

### Set Color
```json
{
  "cmd": "projector",
  "action": "set_color",
  "value": [1.0, 0.8, 0.6]
}
```
- Value: [R, G, B] (0.0-1.0 each)

### Set Pose
```json
{
  "cmd": "projector",
  "action": "set_pose",
  "position": [x, y, z],
  "orientation": [w, x, y, z]
}
```

---

## Ceiling Lights Commands

### Turn On/Off
```json
{
  "cmd": "lights",
  "action": "turn_on",
  "value": true
}
```

### Set Intensity
```json
{
  "cmd": "lights",
  "action": "set_intensity",
  "value": 100.0
}
```

### Set Radius
```json
{
  "cmd": "lights",
  "action": "set_radius",
  "value": 5.0
}
```

### Set FOV
```json
{
  "cmd": "lights",
  "action": "set_fov",
  "value": 90.0
}
```
- Value: Field of view in degrees (0-180)

### Set Color
```json
{
  "cmd": "lights",
  "action": "set_color",
  "value": [1.0, 1.0, 1.0]
}
```

---

## Environment Commands

### Curtains (Extend/Retract)
```json
{
  "cmd": "curtains",
  "action": "extend",
  "value": true
}
```
- `true`: Extend curtains (darken room)
- `false`: Retract curtains (let light in)

### Switch Terrain
```json
{
  "cmd": "terrain",
  "action": "switch",
  "value": 1
}
```
- Value: `0` or `1` (terrain variant)
- **Note:** Triggers world reset

### Enable/Disable Rocks
```json
{
  "cmd": "rocks",
  "action": "enable",
  "value": true
}
```
- **Note:** Triggers world reset

### Randomize Rocks
```json
{
  "cmd": "rocks",
  "action": "randomize",
  "value": 50
}
```
- Value: Number of rocks to place
- **Note:** Triggers world reset

---

## Response Format

All commands return a JSON response:

### Success Response
```json
{
  "success": true,
  "...": "additional fields depending on command"
}
```

### Error Response
```json
{
  "success": false,
  "error": "error description"
}
```

### Robot State Response
```json
{
  "success": true,
  "state": {
    "/husky": {
      "position": [x, y, z],
      "orientation": [x, y, z, w],
      "linear_velocity": [vx, wz],
      "wheel_physics_mode": true
    }
  }
}
```

---

## Example Usage

```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5555))

# Move robot forward
command = {"/husky": {"cmd": "vel", "val": [0.5, 0.0]}}
sock.sendall((json.dumps(command) + "\n").encode())

# Receive response
response = sock.recv(4096)
print(json.loads(response.decode()))

sock.close()
```

---

## Testing

Run the comprehensive test script:
```bash
python clients/tcp_full_features_test.py
```

This will test all available features automatically.
