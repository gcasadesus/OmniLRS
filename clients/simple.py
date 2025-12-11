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

    # Receive state (assuming this is a separate request or the last response)
    # If the server sends state unsolicited, this might need a different approach.
    # If it's a request, you'd send a command like {"cmd": "get_state"} first.
    # For now, assuming the last response *is* the state or we need to explicitly ask.
    # If the server sends state after the vel command, the 'response' above would be the state.
    # If a separate request is needed:
    # command = {"cmd": "get_state"} # Example command
    # state_response = send_command_and_receive_response(sock, command)
    # if state_response:
    #     print(f"Robot state: {state_response}")
    # else:
    #     print("Failed to get robot state.")

except ConnectionRefusedError:
    print(
        "Connection refused. Is the simulator running and listening on 127.0.0.1:5555?"
    )
except Exception as e:
    print(f"An error occurred during connection or communication: {e}")
finally:
    sock.close()
    print("Socket closed.")
