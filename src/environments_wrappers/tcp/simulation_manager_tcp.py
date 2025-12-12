import logging
import socket
import select
import json
import time

from typing import Dict, Any, Tuple
import omni
from src.environments_wrappers.tcp.lunalab_tcp import TCP_LunalabManager
from src.physics.physics_scene import PhysicsSceneManager
from src.configurations.procedural_terrain_confs import TerrainManagerConf
from omegaconf import DictConfig

logger = logging.getLogger(__name__)

HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 5555


from isaacsim.core.api.world import World


class TCP_SimulationManager:
    def __init__(self, cfg: Dict[str, Any], simulation_app) -> None:
        self.cfg = cfg
        self.simulation_app = simulation_app

        # Setup TCP Server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.server_socket.setblocking(False)
        self.client_socket = None
        self.buffer = ""  # Input buffer for partial/multiple packets
        self._outgoing_buffer = b""  # Output buffer for non-blocking writes

        print(f"TCP Server listening on {HOST}:{PORT}", flush=True)

        self.timeline = omni.timeline.get_timeline_interface()
        # Setup Physics and World
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=cfg["environment"]["physics_dt"],
            rendering_dt=cfg["environment"]["rendering_dt"],
        )

        self.physics_manager = PhysicsSceneManager(cfg["physics"]["physics_scene"])

        # Warmup loop (Critical for USDRT/Fabric initialization)
        print("Warming up simulation...", flush=True)
        for i in range(100):
            self.world.step(render=True)
        self.world.reset()
        print("Warmup complete.", flush=True)

        # Initialize Environment Manager based on config
        # Import environment managers
        from src.environments_wrappers.tcp.lunalab_tcp import TCP_LunalabManager
        from src.environments_wrappers.tcp.lunaryard_tcp import TCP_LunaryardManager
        from src.environments_wrappers.tcp.largescale_tcp import TCP_LargeScaleManager

        env_name = cfg["environment"]["name"].lower()

        # Deformation settings
        self.enable_deformation = False
        self.deform_interval = 10
        if "terrain_manager" in cfg["environment"]:
            try:
                # cfg["environment"]["terrain_manager"] is already a TerrainManagerConf object
                tm_conf = cfg["environment"]["terrain_manager"]
                self.enable_deformation = tm_conf.moon_yard.deformation_engine.enable

                # Convert delay (in seconds) to interval (in simulation steps)
                # delay is in seconds, physics_dt is seconds per step
                physics_dt = cfg["environment"].get(
                    "physics_dt", 0.0333
                )  # Default 30 Hz
                delay_seconds = tm_conf.moon_yard.deformation_engine.delay
                self.deform_interval = max(1, int(delay_seconds / physics_dt))

                # TEMP: Reduce deformation frequency for testing
                self.deform_interval *= 10  # Run 10x less often

            except Exception as e:
                logger.warning(f"Failed to load terrain manager config: {e}")

        print(f"=" * 60)
        print(f"DEFORMATION CONFIG CHECK:")
        print(f"  Enabled: {self.enable_deformation}")
        print(f"  Interval: {self.deform_interval}")
        print(f"=" * 60)
        logger.info(f"Deformation Enabled: {self.enable_deformation}")
        logger.info(f"Deformation Interval: {self.deform_interval}")

        if "lunalab" in env_name:
            self.manager = TCP_LunalabManager(
                cfg["environment"], self.physics_manager, self.world
            )
            logger.info("Initialized Lunalab TCP environment")
        elif "lunaryard" in env_name:
            self.manager = TCP_LunaryardManager(
                cfg["environment"], self.physics_manager, self.world
            )
            logger.info("Initialized Lunaryard TCP environment")
        elif "largescale" in env_name or "large_scale" in env_name:
            self.manager = TCP_LargeScaleManager(
                cfg["environment"], self.physics_manager, self.world
            )
            logger.info("Initialized LargeScale TCP environment")
        else:
            raise ValueError(
                f"Environment '{env_name}' not supported in TCP wrapper. "
                f"Supported: lunalab, lunaryard, largescale"
            )

        # Additional step after loading assets
        for i in range(20):
            self.world.step(render=True)
        self.world.reset()

    def run_simulation(self) -> None:
        logger.info("Starting TCP Simulation Loop...")
        self.timeline.play()

        while self.simulation_app.is_running():
            # 1. Handle Network Connections
            self._handle_network()

            # 2. Process incoming commands
            commands = self._get_latest_commands()

            # Consolidate commands to prevent flooding
            # This filters out intermediate velocity commands for the same robot in the current batch
            optimized_commands = self._consolidate_commands(commands)

            # Route commands to appropriate handler
            for command in optimized_commands:
                response = self._process_command(command)

                # Send response back to client
                # We only send response if we actually processed the command (which is true here)
                if response and self.client_socket:
                    self._send_response(response)

            # 3. Flush outgoing buffer
            self._handle_write()

            # 4. Step Physics/Environment (apply persistent forces + deformation)

            # We call step(None) to advance the environment state independent of commands
            state = self.manager.step(None)
            self.simulation_app.update()

            # 5. Deform Terrain (if enabled)
            # MUST be called after simulation step to avoid crashing or stalling the physics engine
            if self.enable_deformation:
                # Access LunalabController specifically if available
                # We assume self.manager has .LC property (Lunalab and Lunaryard do)
                if hasattr(self.manager, "LC") and self.manager.LC:
                    # Using world.current_time_step_index mechanism
                    # Note: simulation_app.update steps the timeline?
                    # We might need to rely on our own counter if world index isn't reliable with app.update()
                    # But assuming world is stepping...
                    # Let's use a try-except to be safe
                    try:
                        # ROS1 uses: current_time_step_index % interval == 0
                        # But current_time_step_index might be 0 always if using app.update without explicit world.step?
                        # Let's check internal counter or just assume world works.
                        # Actually, simulation_app.update() calls IRunLoop::update which ticks the timeline.

                        # Use a dedicated counter to be safe
                        if not hasattr(self, "_deform_step_counter"):
                            self._deform_step_counter = 0

                        if self._deform_step_counter % self.deform_interval == 0:
                            # logger.info(f"Deforming terrain at step {self._deform_step_counter}")  # Debug
                            self.manager.LC.deform_terrain()

                        self._deform_step_counter += 1

                    except Exception as e:
                        logger.warning(f"Deformation failed: {e}")

    def _consolidate_commands(
        self, commands: list[Dict[str, Any]]
    ) -> list[Dict[str, Any]]:
        """
        Optimizes the command batch by keeping only the latest velocity command
        for each robot and discarding intermediate ones.
        """
        if not commands:
            return []

        # If there are few commands, just return them (optimization overhead might not be worth it)
        if len(commands) < 2:
            return commands

        optimized_commands = []

        # Track the index of the last velocity command for each robot
        # Map: robot_name -> index in original 'commands' list
        last_vel_indices = {}

        # First pass: Identify the last velocity command for each robot
        for i, cmd in enumerate(commands):
            # Check for robot velocity command format: {"/robot_name": {"cmd": "vel", ...}}
            # We look for keys starting with "/"
            robot_keys = [k for k in cmd.keys() if k.startswith("/")]
            if robot_keys:
                robot_name = robot_keys[0]
                sub_cmd = cmd[robot_name]
                if isinstance(sub_cmd, dict) and sub_cmd.get("cmd") == "vel":
                    last_vel_indices[robot_name] = i

        # Second pass: Build optimized list
        for i, cmd in enumerate(commands):
            robot_keys = [k for k in cmd.keys() if k.startswith("/")]

            skip = False
            if robot_keys:
                robot_name = robot_keys[0]
                sub_cmd = cmd[robot_name]
                if isinstance(sub_cmd, dict) and sub_cmd.get("cmd") == "vel":
                    # If this is a velocity command but NOT the last one for this robot, skip it
                    if i != last_vel_indices.get(robot_name, -1):
                        skip = True

            if not skip:
                optimized_commands.append(cmd)
            else:
                # logger.debug(f"Skipping superseded velocity command for {robot_keys[0]}")
                pass

        if len(optimized_commands) < len(commands):
            logger.debug(
                f"Consolidated commands from {len(commands)} to {len(optimized_commands)}"
            )

        return optimized_commands

    def _process_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Routes commands to appropriate handlers.

        Args:
            command: Command dictionary from client

        Returns:
            Response dictionary to send back to client
        """
        cmd_type = command.get("cmd")

        # Simulation control commands
        if cmd_type == "pause":
            self.timeline.pause()
            return {"success": True, "state": "paused"}
        elif cmd_type == "play":
            self.timeline.play()
            return {"success": True, "state": "playing"}
        elif cmd_type == "step":
            # Step simulation forward
            # Supports both frame-based and time-based stepping
            num_steps = command.get("steps", None)
            duration = command.get("seconds", None)

            if duration is not None:
                # Time-based stepping: calculate frames from duration
                physics_dt = self.cfg["environment"]["physics_dt"]
                num_steps = int(duration / physics_dt)
                actual_time = num_steps * physics_dt

                for _ in range(num_steps):
                    self.world.step(render=True)

                return {
                    "success": True,
                    "steps": num_steps,
                    "seconds": actual_time,
                    "physics_dt": physics_dt,
                }
            elif num_steps is not None:
                # Frame-based stepping
                for _ in range(num_steps):
                    self.world.step(render=True)

                return {"success": True, "steps": num_steps}
            else:
                # Default: 1 frame
                self.world.step(render=True)
                return {"success": True, "steps": 1}
        elif cmd_type == "get_status":
            is_playing = self.timeline.is_playing()
            return {
                "success": True,
                "playing": is_playing,
                "paused": not is_playing,
                "current_time": self.timeline.get_current_time(),
            }

        # Robot management commands
        elif cmd_type in ["spawn_robot", "teleport", "reset_robot", "reset_all_robots"]:
            return self.manager.robot_manager.handle_robot_command(command)

        # Environment control commands
        elif cmd_type in ["projector", "lights", "curtains", "terrain", "rocks", "sun"]:
            return self.manager.handle_environment_command(command)

        elif cmd_type == "get_state":
            # New explicit command to get state
            return {"success": True, "state": self.manager.robot_manager.get_state()}

        # Robot velocity commands (legacy format: {"/robot_name": {"cmd": "vel", ...}})
        elif any(key.startswith("/") for key in command.keys()):
            # Apply robot velocity command
            self.manager.step(command)

            # Protocol Optimization: For 'vel' commands, return ONLY success
            # Check if any sub-command is 'vel'
            is_vel = False
            for key, val in command.items():
                if isinstance(val, dict) and val.get("cmd") == "vel":
                    is_vel = True
                    break

            if is_vel:
                # Minimal response for high-frequency control
                return {"success": True}
            else:
                # Full state for other robot commands
                return {
                    "success": True,
                    "state": self.manager.robot_manager.get_state(),
                }

        # Wheel mode toggle
        elif cmd_type == "wheel_mode":
            self.manager.robot_manager.set_wheel_physics_mode(command.get("val", True))
            return {
                "success": True,
                "wheel_physics": self.manager.robot_manager.use_wheel_physics,
            }

        # Camera commands
        elif cmd_type == "get_camera_image":
            import cv2
            import base64
            import numpy as np

            robot_name = command.get("robot_name", "husky")
            resolution = command.get("resolution", "low")

            # Helper to find robot key
            target_robot = None
            if hasattr(self.manager, "robot_manager") and self.manager.robot_manager:
                robots = self.manager.robot_manager.robots
                if robot_name in robots:
                    target_robot = robots[robot_name]
                elif "/" + robot_name in robots:
                    target_robot = robots["/" + robot_name]

            if target_robot:
                try:
                    # Get RGBA image
                    img = target_robot.get_rgba_camera_view(resolution)
                    # Ensure it's usually (H, W, 4)

                    # Convert to BGRA for OpenCV if needed, but imencode expects BGR/BGRA usually
                    # Isaac Sim returns RGBA. OpenCV uses BGR.
                    # So we should convert RGBA -> BGRA
                    img_bgra = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)

                    success, encoded_img = cv2.imencode(".png", img_bgra)
                    if success:
                        b64_img = base64.b64encode(encoded_img).decode("utf-8")
                        return {
                            "success": True,
                            "type": "png",
                            "data": b64_img,
                            "shape": img.shape,
                        }
                    else:
                        return {"success": False, "error": "Failed to encode image"}
                except Exception as e:
                    logger.error(f"Error getting camera image: {e}")
                    return {"success": False, "error": str(e)}
            else:
                return {"success": False, "error": f"Robot {robot_name} not found"}

        elif cmd_type == "get_camera_depth":
            import base64
            import numpy as np

            robot_name = command.get("robot_name", "husky")
            resolution = command.get("resolution", "low")

            # Helper to find robot key
            target_robot = None
            if hasattr(self.manager, "robot_manager") and self.manager.robot_manager:
                robots = self.manager.robot_manager.robots
                if robot_name in robots:
                    target_robot = robots[robot_name]
                elif "/" + robot_name in robots:
                    target_robot = robots["/" + robot_name]

            if target_robot:
                try:
                    # Get Depth image (H, W) float32
                    depth = target_robot.get_depth_camera_view(resolution)

                    # Encode as raw bytes in base64
                    depth_bytes = depth.tobytes()
                    b64_data = base64.b64encode(depth_bytes).decode("utf-8")

                    return {
                        "success": True,
                        "type": "npy",
                        "data": b64_data,
                        "params": {"dtype": str(depth.dtype), "shape": depth.shape},
                    }
                except Exception as e:
                    logger.error(f"Error getting camera depth: {e}")
                    return {"success": False, "error": str(e)}
            else:
                return {"success": False, "error": f"Robot {robot_name} not found"}

        # Unknown command
        else:
            return {"success": False, "error": f"Unknown command type: {cmd_type}"}

    def _handle_write(self) -> None:
        """
        Flushes the outgoing buffer to the socket non-blocking.
        Must be called periodically.
        """
        if not self.client_socket or not self._outgoing_buffer:
            return

        try:
            # Try to send everything
            sent = self.client_socket.send(self._outgoing_buffer)
            if sent > 0:
                self._outgoing_buffer = self._outgoing_buffer[sent:]
        except BlockingIOError:
            # Socket buffer full, try again later
            pass
        except OSError as e:
            logger.error(f"Socket write error: {e}")
            self.client_socket.close()
            self.client_socket = None
            self._outgoing_buffer = b""

    def _handle_network(self) -> None:
        """Accept new connections."""
        try:
            readable, _, _ = select.select([self.server_socket], [], [], 0.0)
            if self.server_socket in readable:
                client, addr = self.server_socket.accept()
                logger.info(f"Accepted connection from {addr}")
                client.setblocking(False)  # Non-blocking mode
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except OSError:
                        pass
                self.client_socket = client
                self.buffer = ""
                self._outgoing_buffer = b""  # Clear outgoing buffer on new connection
        except OSError:
            pass

    def _get_latest_commands(self) -> list[Dict[str, Any]]:
        """Read from socket and return list of available commands.
        Reads all available data until blocking to maximize batching.
        """
        if not self.client_socket:
            return []

        commands = []
        try:
            while True:
                try:
                    data = self.client_socket.recv(4096)
                    if not data:
                        logger.info("Client disconnected.")
                        self.client_socket.close()
                        self.client_socket = None
                        self.buffer = ""
                        self._outgoing_buffer = (
                            b""  # Clear outgoing buffer on disconnect
                        )
                        return []

                    self.buffer += data.decode("utf-8")

                except BlockingIOError:
                    # No more data to read for now
                    break
                except ConnectionResetError:
                    logger.info("Client connection reset.")
                    self.client_socket.close()
                    self.client_socket = None
                    self.buffer = ""
                    self._outgoing_buffer = b""  # Clear outgoing buffer on disconnect
                    return []

            # Process buffer
            if "\n" in self.buffer:
                lines = self.buffer.split("\n")
                # Process all complete lines
                for line in lines[:-1]:
                    line = line.strip()
                    if line:
                        try:
                            cmd = json.loads(line)
                            # logger.debug(f"Received command: {cmd}") # Verbose
                            commands.append(cmd)
                        except json.JSONDecodeError:
                            logger.warning(f"Received invalid JSON line: {line}")

                # Keep the last partial line in buffer
                self.buffer = lines[-1]

        except OSError as e:
            logger.error(f"Socket error: {e}")
            self.client_socket = None
            self.buffer = ""
            self._outgoing_buffer = b""  # Clear outgoing buffer on error

        return commands

    def _send_state(self, state: Dict[str, Any]) -> None:
        """Send state as JSON to client (legacy method)."""
        if not self.client_socket:
            return

        try:
            msg = json.dumps(state) + "\n"
            self.client_socket.sendall(msg.encode("utf-8"))
        except BlockingIOError:
            logger.warning("TCP send buffer full (state dropped)")
        except OSError as e:
            logger.warning(f"Failed to send state: {e}")
            self.client_socket.close()
            self.client_socket = None

    def _send_response(self, response: Dict[str, Any]) -> None:
        """Queue response to be sent to client."""
        if not self.client_socket:
            return

        try:
            msg = json.dumps(response) + "\n"
            data = msg.encode("utf-8")

            # Append to buffer instead of direct sendall
            self._outgoing_buffer += data

            # Try to flush immediately
            self._handle_write()

        except Exception as e:
            logger.warning(f"Failed to queue response: {e}")
