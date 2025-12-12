# OmniLRS TCP Server - Complete API Reference

**Version:** 2.0  
**Last Updated:** 2025-12-11

## Table of Contents
- [Overview](#overview)
- [Connection](#connection)
- [Supported Environments](#supported-environments)
- [Simulation Control](#simulation-control)
- [Robot Management](#robot-management)
- [Environment-Specific Commands](#environment-specific-commands)
  - [Lunalab Commands](#lunalab-commands)
  - [Lunaryard Commands](#lunaryard-commands)
  - [LargeScale Commands](#largescale-commands)
- [Response Format](#response-format)
- [Code Examples](#code-examples)

---

## Overview

The OmniLRS TCP Server provides complete control over Isaac Sim-based lunar simulation environments via TCP sockets. It supports:

- ✅ **3 environment types** (Lunalab, Lunaryard, LargeScale)
- ✅ **Deformable terrain** variants
- ✅ **30+ control commands**
- ✅ **Real-time robot control** with wheel physics
- ✅ **Environment manipulation** (lighting, terrain, rocks)
- ✅ **Simulation control** (pause, play, step)

---

## Connection

### Server Details
- **Host:** `0.0.0.0` (listens on all interfaces)
- **Port:** `5555`
- **Protocol:** TCP with JSON messages
- **Message Format:** JSON string terminated by `\n`

### Python Example
```python
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5555))

# Send command
command = {"cmd": "pause"}
sock.sendall((json.dumps(command) + "\n").encode())

# Receive response
data = sock.recv(4096)
response = json.loads(data.decode())
print(response)  # {"success": true, "state": "paused"}

sock.close()
```

---

## Supported Environments

### Start Server with Specific Environment

```bash
# Lunalab (indoor lab)
./run_tcp.sh display lunalab

# Lunaryard variants
./run_tcp.sh display lunaryard_20m
./run_tcp.sh display lunaryard_40m
./run_tcp.sh display lunaryard_80m

# Large scale terrain
./run_tcp.sh display largescale

# Deformable terrain variants
./run_tcp.sh display lunalab_deformable
./run_tcp.sh display lunaryard_40m_deformable
```

### Environment Features

| Environment | Lighting | Terrain Switch | Rocks | Deformation |
|-------------|----------|----------------|-------|-------------|
| **Lunalab** | Projector + Ceiling Lights | ✅ | ✅ | ✅ (variant) |
| **Lunaryard** | Sun | ✅ | ✅ | ✅ (variant) |
| **LargeScale** | Sun | ❌ | ❌ | ❌ |

---

## Simulation Control

### Pause Simulation
```json
{"cmd": "pause"}
```
**Response:**
```json
{"success": true, "state": "paused"}
```

### Play/Resume Simulation
```json
{"cmd": "play"}
```
**Response:**
```json
{"success": true, "state": "playing"}
```

### Step Simulation
Step forward by N frames (useful when paused):
```json
{"cmd": "step", "steps": 10}
```
**Response:**
```json
{"success": true, "steps": 10}
```

### Get Simulation Status
```json
{"cmd": "get_status"}
```
**Response:**
```json
{
  "success": true,
  "playing": false,
  "paused": true,
  "current_time": 123.456
}
```

---

### Get Simulation Status
```json
{"cmd": "get_status"}
```
**Response:**
```json
{
  "success": true,
  "playing": false,
  "paused": true,
  "current_time": 123.456
}
```

### Get Robot State
To retrieve the full state (pose, velocity) of all robots:
```json
{"cmd": "get_state"}
```
**Response:**
```json
{
  "success": true,
  "state": {
    "/husky": {
      "position": [5.0, 5.0, 0.5],
      "orientation": [0, 0, 0, 1],
      "linear_velocity": [0.5, 0.0]
    }
  }
}
```

---


### Robot Velocity Control
**Persistent velocity commands** - send once, robot continues moving until stopped.

```json
{
  "/husky": {
    "cmd": "vel",
    "val": [0.5, 0.0]
  }
}
```
- `val[0]`: Linear velocity (m/s)
- `val[1]`: Angular velocity (rad/s)
- **Response:** `{"success": true}` (Minimal response for high performance)
- **Default:** Wheel physics enabled (wheels rotate)


**Stop Robot:**
```json
{"/husky": {"cmd": "vel", "val": [0.0, 0.0]}}
```

### Teleport Robot
```json
{
  "cmd": "teleport",
  "robot_name": "/husky",
  "position": [5.0, 5.0, 0.5],
  "orientation": [1, 0, 0, 0]
}
```
- `position`: [x, y, z] in meters
- `orientation`: [w, x, y, z] quaternion

### Spawn Robot
```json
{
  "cmd": "spawn_robot",
  "robot_name": "/husky2",
  "usd_path": "assets/USD_Assets/robots/ros2_husky_PhysX_vlp16.usd",
  "position": [0, 0, 0.5],
  "orientation": [1, 0, 0, 0]
}
```

### Reset Robot
```json
{"cmd": "reset_robot", "robot_name": "/husky"}
```

### Reset All Robots
```json
{"cmd": "reset_all_robots"}
```

### Toggle Wheel Physics Mode
```json
{"cmd": "wheel_mode", "val": true}
```
- `true`: Wheel-based physics (realistic, wheels rotate)
- `false`: Root-based driving (simpler, no wheel rotation)

---

### Camera Control
Retrieve sensor data from robot's cameras.

**Get RGB Image:**
```json
{
  "cmd": "get_camera_image",
  "robot_name": "/husky",
  "camera": "left",
  "resolution": "high"
}
```
**Response:**
```json
{
  "success": true,
  "type": "png",
  "data": "<base64_encoded_png_data>",
  "shape": [720, 1280, 4]
}
```

**Get Depth Map:**
```json
{
  "cmd": "get_camera_depth",
  "robot_name": "/husky",
  "camera": "left",
  "resolution": "high"
}
```
**Response:**
```json
{
  "success": true,
  "type": "npy",
  "data": "<base64_encoded_float32_bytes>",
  "params": {
    "dtype": "float32",
    "shape": [720, 1280]
  }
}
```

---

## Environment-Specific Commands

### Lunalab Commands

#### Projector Control

**Turn On/Off:**
```json
{"cmd": "projector", "action": "turn_on", "value": true}
```

**Set Intensity (0-100%):**
```json
{"cmd": "projector", "action": "set_intensity", "value": 75.0}
```

**Set Radius (meters):**
```json
{"cmd": "projector", "action": "set_radius", "value": 2.5}
```

**Set Color (RGB 0-1):**
```json
{"cmd": "projector", "action": "set_color", "value": [1.0, 0.8, 0.6]}
```

**Set Pose:**
```json
{
  "cmd": "projector",
  "action": "set_pose",
  "position": [0, 0, 3],
  "orientation": [1, 0, 0, 0]
}
```

#### Ceiling Lights Control

**Turn On/Off:**
```json
{"cmd": "lights", "action": "turn_on", "value": true}
```

**Set Intensity:**
```json
{"cmd": "lights", "action": "set_intensity", "value": 100.0}
```

**Set Radius:**
```json
{"cmd": "lights", "action": "set_radius", "value": 5.0}
```

**Set FOV (0-180°):**
```json
{"cmd": "lights", "action": "set_fov", "value": 90.0}
```

**Set Color:**
```json
{"cmd": "lights", "action": "set_color", "value": [1.0, 1.0, 1.0]}
```

#### Curtains
```json
{"cmd": "curtains", "action": "extend", "value": true}
```
- `true`: Extend (darken room)
- `false`: Retract (let light in)

#### Terrain & Rocks
```json
{"cmd": "terrain", "action": "switch", "value": 1}
```
- `value`: 0 or 1 (terrain variant)
- **Note:** Triggers world reset

```json
{"cmd": "rocks", "action": "enable", "value": true}
```

```json
{"cmd": "rocks", "action": "randomize", "value": 50}
```
- `value`: Number of rocks to place

---

### Lunaryard Commands

Lunaryard uses **sun** instead of projector/lights.

#### Sun Control

**Set Intensity:**
```json
{"cmd": "sun", "action": "set_intensity", "value": 1000.0}
```

**Set Color (RGB 0-1):**
```json
{"cmd": "sun", "action": "set_color", "value": [1.0, 0.95, 0.8]}
```

**Set Color Temperature (Kelvin):**
```json
{"cmd": "sun", "action": "set_color_temperature", "value": 5800.0}
```

**Set Angle (affects shadow softness):**
```json
{"cmd": "sun", "action": "set_angle", "value": 0.53}
```

**Set Pose:**
```json
{
  "cmd": "sun",
  "action": "set_pose",
  "position": [1000, 1000, 1000],
  "orientation": [1, 0, 0, 0]
}
```

#### Terrain & Rocks
Same as Lunalab (see above).

---

### LargeScale Commands

LargeScale only supports **sun** controls (same as Lunaryard).

No terrain switching or rocks management available.

---

## Response Format

### Success Response
```json
{
  "success": true,
  "...": "additional fields"
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
(Response to `{"cmd": "get_state"}`)
```json
{
  "success": true,
  "state": {
    "/husky": {
      "position": [5.0, 5.0, 0.5],
      "orientation": [0, 0, 0, 1],
      "linear_velocity": [0.5, 0.0],
      "wheel_physics_mode": true
    }
  }
}
```


---

## Code Examples

### Complete Control Flow

```python
import socket
import json
import time

def send_command(sock, cmd, description=""):
    if description:
        print(f"\n{description}")
    sock.sendall((json.dumps(cmd) + "\n").encode())
    time.sleep(0.1)
    try:
        sock.settimeout(1.0)
        data = sock.recv(4096)
        if data:
            print(f"Response: {json.loads(data.decode())}")
    except socket.timeout:
        pass

# Connect
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5555))

# Pause simulation
send_command(sock, {"cmd": "pause"}, "Pausing simulation")

# Teleport robot
send_command(sock, {
    "cmd": "teleport",
    "robot_name": "/husky",
    "position": [3, 3, 0.5],
    "orientation": [1, 0, 0, 0]
}, "Teleporting robot")

# Resume simulation
send_command(sock, {"cmd": "play"}, "Resuming simulation")

# Move robot forward
send_command(sock, {
    "/husky": {"cmd": "vel", "val": [0.3, 0.0]}
}, "Moving forward")

time.sleep(3)

# Stop robot
send_command(sock, {
    "/husky": {"cmd": "vel", "val": [0.0, 0.0]}
}, "Stopping")

# Adjust lighting (Lunalab only)
send_command(sock, {
    "cmd": "lights",
    "action": "set_intensity",
    "value": 75.0
}, "Dimming lights")

sock.close()
```

### Comprehensive Test Script

See `clients/tcp_full_features_test.py` for a complete example testing all features.

---

## Quick Reference

### Most Common Commands

| Action | Command |
|--------|---------|
| **Move robot forward** | `{"/husky": {"cmd": "vel", "val": [0.5, 0.0]}}` |
| **Rotate robot** | `{"/husky": {"cmd": "vel", "val": [0.0, 0.5]}}` |
| **Stop robot** | `{"/husky": {"cmd": "vel", "val": [0.0, 0.0]}}` |
| **Pause** | `{"cmd": "pause"}` |
| **Play** | `{"cmd": "play"}` |
| **Teleport** | `{"cmd": "teleport", "robot_name": "/husky", "position": [x,y,z], "orientation": [w,x,y,z]}` |
| **Reset robot** | `{"cmd": "reset_robot", "robot_name": "/husky"}` |
| **Switch terrain** | `{"cmd": "terrain", "action": "switch", "value": 1}` |
| **Lights on** | `{"cmd": "lights", "action": "turn_on", "value": true}` (Lunalab) |
| **Sun intensity** | `{"cmd": "sun", "action": "set_intensity", "value": 1000.0}` (Lunaryard/LargeScale) |

---

## Notes

- **Persistent commands:** Velocity commands persist until changed or stopped with `[0.0, 0.0]`
- **World resets:** Terrain switching and rock changes trigger automatic world resets
- **Wheel physics:** Enabled by default for realistic motion
- **Multiple robots:** All commands support multiple robots (use different robot_name)
- **Deformable terrain:** Automatically enabled when using `*_deformable` configs

---

## Support

For issues and questions:
- **GitHub:** [OmniLRS Repository](https://github.com/your-repo)
- **Documentation:** See `docs/` directory
- **Examples:** See `clients/` directory

---

**Happy Simulating! 🌙🤖**
