import logging
from typing import Dict, Any, Tuple

from src.environments.lunalab import LunalabController
from src.environments_wrappers.tcp.robot_manager_tcp import TCP_RobotManager
from src.physics.physics_scene import PhysicsSceneManager
from src.robots.robot import RobotManager
from isaacsim.core.api.world import World

logger = logging.getLogger(__name__)


class TCP_LunalabManager:
    def __init__(
        self, cfg: Dict[str, Any], physics_manager: PhysicsSceneManager, world: World
    ) -> None:
        self.physics_manager = physics_manager
        self.world = world

        # Initialize the Environment Controller
        # cfg should be the 'environment' section of the config
        self.LC = LunalabController(**cfg)
        self.LC.load()

        # Create RobotManager and preload robots (mimicking ROS2 pattern)
        self.RM = RobotManager(cfg["robots_settings"])
        self.RM.preload_robot(self.world)

        # Add RobotManager to LunalabController
        self.LC.add_robot_manager(self.RM)

        # Initialize the TCP Robot Manager with RobotManager reference
        self.robot_manager = TCP_RobotManager(
            robots=self.RM.robots,
            physics_manager=self.physics_manager,
            robot_manager=self.RM,
        )
        self.deform_counter = 0
        self.deform_interval = 10  # Update deformation every 10 steps

        logger.info("TCP Lunalab Manager initialized.")

    def handle_environment_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handles environment control commands (lighting, terrain, rocks, curtains).

        Args:
            command: Command dictionary

        Returns:
            Response dictionary with success status
        """
        cmd_type = command.get("cmd")
        action = command.get("action")
        value = command.get("value")

        try:
            if cmd_type == "projector":
                return self._handle_projector_command(action, value, command)
            elif cmd_type == "lights":
                return self._handle_lights_command(action, value, command)
            elif cmd_type == "curtains":
                return self._handle_curtains_command(action, value)
            elif cmd_type == "terrain":
                return self._handle_terrain_command(action, value)
            elif cmd_type == "rocks":
                return self._handle_rocks_command(action, value)
            else:
                return {
                    "success": False,
                    "error": f"Unknown environment command: {cmd_type}",
                }
        except Exception as e:
            logger.error(f"Error handling environment command {cmd_type}/{action}: {e}")
            return {"success": False, "error": str(e)}

    def _handle_projector_command(
        self, action: str, value: Any, command: Dict
    ) -> Dict[str, Any]:
        """Handle projector-related commands"""
        try:
            if action == "turn_on":
                self.LC.turn_projector_on_off(flag=bool(value))
                return {"success": True, "action": "turn_on", "value": value}
            elif action == "set_intensity":
                # Scale to default intensity (from ROS2 implementation)
                default_intensity = 300000000.0
                intensity = default_intensity * float(value) / 100.0
                self.LC.set_projector_intensity(intensity=intensity)
                return {"success": True, "action": "set_intensity", "value": value}
            elif action == "set_radius":
                self.LC.set_projector_radius(radius=float(value))
                return {"success": True, "action": "set_radius", "value": value}
            elif action == "set_color":
                # value should be [r, g, b]
                self.LC.set_projector_color(color=tuple(value))
                return {"success": True, "action": "set_color", "value": value}
            elif action == "set_pose":
                position = command.get("position", [0, 0, 0])
                orientation = command.get("orientation", [1, 0, 0, 0])
                self.LC.set_projector_pose(
                    position=tuple(position), orientation=tuple(orientation)
                )
                return {"success": True, "action": "set_pose", "position": position}
            else:
                return {
                    "success": False,
                    "error": f"Unknown projector action: {action}",
                }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _handle_lights_command(
        self, action: str, value: Any, command: Dict
    ) -> Dict[str, Any]:
        """Handle ceiling lights commands"""
        try:
            if action == "turn_on":
                self.LC.turn_room_lights_on_off(flag=bool(value))
                return {"success": True, "action": "turn_on", "value": value}
            elif action == "set_intensity":
                self.LC.set_room_lights_intensity(intensity=float(value))
                return {"success": True, "action": "set_intensity", "value": value}
            elif action == "set_radius":
                self.LC.set_room_lights_radius(radius=float(value))
                return {"success": True, "action": "set_radius", "value": value}
            elif action == "set_fov":
                self.LC.set_room_lights_FOV(FOV=float(value))
                return {"success": True, "action": "set_fov", "value": value}
            elif action == "set_color":
                # value should be [r, g, b]
                self.LC.set_room_lights_color(color=tuple(value))
                return {"success": True, "action": "set_color", "value": value}
            else:
                return {"success": False, "error": f"Unknown lights action: {action}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _handle_curtains_command(self, action: str, value: Any) -> Dict[str, Any]:
        """Handle curtains commands"""
        try:
            if action == "extend":
                self.LC.curtains_extend(flag=bool(value))
                return {"success": True, "action": "extend", "value": value}
            else:
                return {"success": False, "error": f"Unknown curtains action: {action}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _handle_terrain_command(self, action: str, value: Any) -> Dict[str, Any]:
        """Handle terrain commands"""
        try:
            if action == "switch":
                self.LC.switch_terrain(flag=int(value))
                # Trigger world reset after terrain switch
                self.world.reset()
                return {"success": True, "action": "switch", "value": value}
            else:
                return {"success": False, "error": f"Unknown terrain action: {action}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _handle_rocks_command(self, action: str, value: Any) -> Dict[str, Any]:
        """Handle rocks commands"""
        try:
            if action == "enable":
                self.LC.enable_rocks(flag=bool(value))
                # Trigger world reset after rocks change
                self.world.reset()
                return {"success": True, "action": "enable", "value": value}
            elif action == "randomize":
                num_rocks = int(value)
                self.LC.randomize_rocks(num=num_rocks)
                # Trigger world reset after randomization
                self.world.reset()
                return {"success": True, "action": "randomize", "value": num_rocks}
            else:
                return {"success": False, "error": f"Unknown rocks action: {action}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def step(self, command_dict: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Called every simulation step.
        Applies new commands if provided, then reapplies persistent velocity commands.
        """
        # Apply new commands if provided (updates stored velocity)
        if command_dict:
            for robot_name, cmd in command_dict.items():
                self.robot_manager.apply_command(robot_name, cmd)

        # Apply persistent velocity commands every frame
        self.robot_manager.apply_persistent_commands()

        # Update terrain deformation if enabled
        # MOVED TO SIMULATION_MANAGER MAIN LOOP FOR PROPER SCHEDULING
        # if self.LC.deformation_conf and self.LC.deformation_conf.enable:
        #     if self.deform_counter % self.deform_interval == 0:
        #         self.LC.deform_terrain()
        #     self.deform_counter += 1

        # Return state
        return self.robot_manager.get_state()
