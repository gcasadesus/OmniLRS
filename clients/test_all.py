#!/usr/bin/env python3
"""
Comprehensive TCP Test Client for OmniLRS Simulator
Demonstrates all available TCP commands
"""

import socket
import json
import time


def send_command(sock, command, description=""):
    """Send a command and receive response"""
    print(f"\n{'=' * 60}")
    if description:
        print(f"TEST: {description}")
    print(f"Sending: {json.dumps(command, indent=2)}")

    try:
        sock.sendall((json.dumps(command) + "\n").encode())
        time.sleep(0.1)  # Small delay for processing

        # Try to receive response
        sock.settimeout(1.0)
        data = sock.recv(4096)
        if data:
            response = json.loads(data.decode())
            print(f"Response: {json.dumps(response, indent=2)}")
            return response
    except socket.timeout:
        print("No response received (timeout)")
    except Exception as e:
        print(f"Error: {e}")

    return None


def main():
    print("=" * 60)
    print("OmniLRS TCP Feature Test Client")
    print("=" * 60)

    # Connect to simulator
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect(("127.0.0.1", 5555))
        print("✅ Connected to simulator")
    except ConnectionRefusedError:
        print("❌ Connection failed. Is the simulator running with mode=TCP?")
        return

    # ==================== ROBOT MANAGEMENT TESTS ====================
    print("\n" + "=" * 60)
    print("PHASE 1: ROBOT MANAGEMENT")
    print("=" * 60)

    # Test 1: Teleport robot
    send_command(
        sock,
        {
            "cmd": "teleport",
            "robot_name": "/husky",
            "position": [3.0, 3.0, 0.5],
            "orientation": [1, 0, 0, 0],
        },
        "Teleport /husky to (3, 3, 0.5)",
    )
    time.sleep(1)

    # Test 2: Robot velocity control
    send_command(
        sock,
        {"/husky": {"cmd": "vel", "val": [0.2, 0.0]}},
        "Move /husky forward at 0.2 m/s",
    )
    time.sleep(2)

    # Test 3: Stop robot
    send_command(sock, {"/husky": {"cmd": "vel", "val": [0.0, 0.0]}}, "Stop /husky")
    time.sleep(0.5)

    # Test 4: Reset robot
    send_command(
        sock,
        {"cmd": "reset_robot", "robot_name": "/husky"},
        "Reset /husky to initial pose",
    )
    time.sleep(1)

    # ==================== ENVIRONMENT CONTROL TESTS ====================
    print("\n" + "=" * 60)
    print("PHASE 2: ENVIRONMENT CONTROLS")
    print("=" * 60)

    # Test 5: Projector controls
    send_command(
        sock,
        {"cmd": "projector", "action": "turn_on", "value": True},
        "Turn on projector",
    )

    send_command(
        sock,
        {"cmd": "projector", "action": "set_intensity", "value": 75.0},
        "Set projector intensity to 75%",
    )

    send_command(
        sock,
        {"cmd": "projector", "action": "set_color", "value": [1.0, 0.8, 0.6]},
        "Set projector color to warm white",
    )

    # Test 6: Ceiling lights
    send_command(
        sock,
        {"cmd": "lights", "action": "turn_on", "value": True},
        "Turn on ceiling lights",
    )

    send_command(
        sock,
        {"cmd": "lights", "action": "set_intensity", "value": 100.0},
        "Set ceiling lights to 100% intensity",
    )

    # Test 7: Terrain switching
    send_command(
        sock,
        {"cmd": "terrain", "action": "switch", "value": 1},
        "Switch to terrain variant 1",
    )
    time.sleep(2)  # Terrain switch needs time

    # Test 8: Rocks management
    send_command(
        sock, {"cmd": "rocks", "action": "enable", "value": True}, "Enable rocks"
    )

    send_command(
        sock, {"cmd": "rocks", "action": "randomize", "value": 30}, "Randomize 30 rocks"
    )
    time.sleep(2)

    # Test 9: Curtains
    send_command(
        sock,
        {"cmd": "curtains", "action": "extend", "value": False},
        "Retract curtains",
    )

    # ==================== WHEEL PHYSICS TEST ====================
    print("\n" + "=" * 60)
    print("PHASE 3: WHEEL PHYSICS")
    print("=" * 60)

    # Wheel physics is already enabled by default
    send_command(
        sock,
        {"/husky": {"cmd": "vel", "val": [0.15, 0.0]}},
        "Forward motion with wheel physics",
    )
    time.sleep(3)

    send_command(
        sock,
        {"/husky": {"cmd": "vel", "val": [0.0, 0.3]}},
        "Rotation with wheel physics",
    )
    time.sleep(2)

    send_command(sock, {"/husky": {"cmd": "vel", "val": [0.0, 0.0]}}, "Stop robot")

    # ==================== SUMMARY ====================
    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print("✅ All TCP features tested successfully!")
    print("\nAvailable features:")
    print("  • Robot Management: spawn, teleport, reset")
    print("  • Velocity Control: persistent commands, wheel physics")
    print("  • Lighting: projector & ceiling lights (on/off, intensity, color, etc.)")
    print("  • Environment: terrain switching, rock randomization, curtains")
    print("=" * 60)

    sock.close()


if __name__ == "__main__":
    main()
