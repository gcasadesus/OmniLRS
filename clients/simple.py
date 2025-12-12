import socket
import json
import time


def send_command_and_receive_response(sock, command):
    """Sends a JSON command and receives a JSON response."""
    try:
        # Send command
        sock.sendall((json.dumps(command) + "\n").encode())

        # Receive response
        # It's crucial to receive data after each send if the server
        # is expected to respond or if it closes the connection otherwise.
        # A small delay might also be necessary for the server to process.
        # For simplicity, we'll try to receive immediately.
        # In a real-world scenario, you might need a loop to read until a newline
        # or a specific message length, and handle partial reads.
        data = sock.recv(4096)
        if not data:
            print("Server closed connection or sent no data.")
            return None

        response = json.loads(data.decode())
        return response
    except BrokenPipeError:
        print("Connection broken. Server might have closed the connection.")
        return None
    except json.JSONDecodeError as e:
        print(f"Failed to decode JSON response: {e}. Received data: {data.decode()}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None


# Connect to simulator
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.connect(("127.0.0.1", 5555))
    print("Connected to simulator.")

    # First, enable wheel physics
    command = {"cmd": "wheel_mode", "val": True}
    response = send_command_and_receive_response(sock, command)
    if response:
        print(f"Wheel mode command response: {response}")
    else:
        print("Failed to enable wheel physics or get response.")
        # Depending on the protocol, you might want to exit here.

    # Then send velocity commands - wheels will now rotate!
    command = {"/husky": {"cmd": "vel", "val": [-0.5, -1.5]}}
    response = send_command_and_receive_response(sock, command)
    if response:
        print(f"Velocity command response: {response}")
    else:
        print("Failed to send velocity command or get response.")

    # Receive state (New API: Explicit request needed)
    # The velocity command now returns minimal response for performance.
    # To get the state, we send a separate get_state command.
    command = {"cmd": "get_state"}
    state_response = send_command_and_receive_response(sock, command)
    if state_response:
        print(f"Robot state: {state_response}")
    else:
        print("Failed to get robot state.")

    # Keep the script running to observe the robot moving
    print("\nRobot is moving... keeping connection open for 5 seconds.")
    for i in range(10):
        time.sleep(1)
        # Optional: Poll state to show it changing
        state = send_command_and_receive_response(sock, {"cmd": "get_state"})
        if state:
            velocity = (
                state.get("state", {}).get("/husky", {}).get("linear_velocity", "N/A")
            )
            print(f"t={i + 1}s: Linear Velocity = {velocity}")

    # Stop the robot before exiting
    print("\nStopping robot...")
    command = {"/husky": {"cmd": "vel", "val": [0.0, 0.0]}}
    send_command_and_receive_response(sock, command)

except ConnectionRefusedError:
    print(
        "Connection refused. Is the simulator running and listening on 127.0.0.1:5555?"
    )
except Exception as e:
    print(f"An error occurred during connection or communication: {e}")
finally:
    sock.close()
    print("Socket closed.")
