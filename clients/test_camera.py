import socket
import json
import base64
import numpy as np
import cv2
import os


def test_camera():
    host = "127.0.0.1"
    port = 5555

    # Request high resolution image and depth
    commands = [
        {
            "cmd": "get_camera_image",
            "resolution": "high",
            "camera": "left",
            "robot_name": "husky",
        },
        {
            "cmd": "get_camera_depth",
            "resolution": "high",
            "camera": "left",
            "robot_name": "husky",
        },
    ]

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))

        for cmd in commands:
            print(f"Sending command: {cmd}")
            s.send(json.dumps(cmd).encode("utf-8"))

            # Receive response (line delimited JSON)
            response_data = b""
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    break
                response_data += chunk
                if b"\n" in chunk:
                    break

            try:
                response = json.loads(response_data.decode("utf-8").strip())
                print(f"Response: {response}")

                if not response.get("success"):
                    print(f"Command failed: {response.get('error')}")
                    continue

                # Decode data if present
                if "data" in response:
                    data = base64.b64decode(response["data"])

                    if cmd["cmd"] == "get_camera_image":
                        with open(f"test_image.png", "wb") as f:
                            f.write(data)
                        print(f"Saved image to test_image.png")
                    elif cmd["cmd"] == "get_camera_depth":
                        # Reconstruct numpy array from base64 decoded data
                        params = response.get(
                            "params", {}
                        )  # Get params from response, if available
                        dtype = np.dtype(
                            params.get("dtype", "float32")
                        )  # Default to float32 if not specified
                        shape = tuple(
                            params.get("shape", ())
                        )  # Default to empty tuple if not specified

                        arr = np.frombuffer(data, dtype=dtype)
                        if shape:  # Reshape only if shape is provided
                            arr = arr.reshape(shape)

                        np.save("test_depth.npy", arr)
                        print(f"Saved test_depth.npy with shape {arr.shape}")

                        # Normalize for visualization
                        depth_vis = cv2.normalize(
                            arr, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U
                        )
                        cv2.imwrite("test_depth_vis.png", depth_vis)
                        print("Saved test_depth_vis.png")
            except json.JSONDecodeError:
                print(f"Failed to decode response: {response_data}")

        s.close()

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    test_camera()
