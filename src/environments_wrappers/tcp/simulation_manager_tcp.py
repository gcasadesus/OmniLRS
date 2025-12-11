import logging
import socket
import select
import json
import time

from typing import Dict, Any
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
            current_command = self._get_latest_command()

            # Route command to appropriate handler
            if current_command:
                response = self._process_command(current_command)
                # Send response back to client
                if response and self.client_socket:
                    self._send_response(response)

            # 3. Step Physics/Environment (for robot velocity commands)
            state = self.manager.step(
                None
            )  # Robot velocity commands are already applied
            self.simulation_app.update()

            # 4. Send state back (only for state requests or periodic updates)
            # For now, we send state after each command
            # This could be optimized to only send on request

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

        # Robot velocity commands (legacy format: {"/robot_name": {"cmd": "vel", ...}})
        elif any(key.startswith("/") for key in command.keys()):
            # Apply robot velocity command
            self.manager.step(command)
            # Return state as response
            return {"success": True, "state": self.manager.robot_manager.get_state()}

        # Wheel mode toggle
        elif cmd_type == "wheel_mode":
            self.manager.robot_manager.set_wheel_physics_mode(command.get("val", True))
            return {
                "success": True,
                "wheel_physics": self.manager.robot_manager.use_wheel_physics,
            }

        # Unknown command
        else:
            return {"success": False, "error": f"Unknown command type: {cmd_type}"}

    def _handle_network(self):
        """Accept new connections."""
        try:
            readable, _, _ = select.select([self.server_socket], [], [], 0.0)
            if self.server_socket in readable:
                client, addr = self.server_socket.accept()
                logger.info(f"Accepted connection from {addr}")
                client.setblocking(False)
                if self.client_socket:
                    self.client_socket.close()  # Only support one client for simplicity
                self.client_socket = client
        except OSError:
            pass

    def _get_latest_command(self) -> Dict[str, Any]:
        """Read from socket if data available."""
        if not self.client_socket:
            return {}

        try:
            readable, _, _ = select.select([self.client_socket], [], [], 0.0)
            if self.client_socket in readable:
                data = self.client_socket.recv(4096)
                if not data:
                    logger.info("Client disconnected.")
                    self.client_socket.close()
                    self.client_socket = None
                    return {}

                # Parse JSON
                # This is a naive implementation that assumes full JSON in one packet
                # Real implementation needs a buffer and delimiters
                try:
                    msg = data.decode("utf-8").strip()
                    command = json.loads(msg)
                    logger.info(f"Received command: {command}")
                    return command
                except json.JSONDecodeError:
                    logger.warning(f"Received invalid JSON: {data}")
                    return {}
        except OSError as e:
            logger.error(f"Socket error: {e}")
            self.client_socket = None

        return {}

    def _send_state(self, state: Dict[str, Any]) -> None:
        """Send state as JSON to client (legacy method)."""
        if not self.client_socket:
            return

        try:
            msg = json.dumps(state) + "\n"
            self.client_socket.sendall(msg.encode("utf-8"))
        except OSError as e:
            logger.warning(f"Failed to send state: {e}")
            self.client_socket.close()
            self.client_socket = None

    def _send_response(self, response: Dict[str, Any]) -> None:
        """Send response as JSON to client."""
        if not self.client_socket:
            return

        try:
            msg = json.dumps(response) + "\n"
            self.client_socket.sendall(msg.encode("utf-8"))
            logger.debug(f"Sent response: {response}")
        except OSError as e:
            logger.warning(f"Failed to send response: {e}")
            self.client_socket.close()
            self.client_socket = None
