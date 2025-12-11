import logging
from typing import Dict, Any

from src.environments.lunaryard import LunaryardController
from src.environments_wrappers.tcp.robot_manager_tcp import TCP_RobotManager
from src.physics.physics_scene import PhysicsSceneManager
from src.robots.robot import RobotManager
from isaacsim.core.api.world import World

logger = logging.getLogger(__name__)


class TCP_LunaryardManager:
    def __init__(
        self, cfg: Dict[str, Any], physics_manager: PhysicsSceneManager, world: World
    ) -> None:
        self.physics_manager = physics_manager
        self.world = world

        # Initialize the Environment Controller
        self.LC = LunaryardController(**cfg)
        self.LC.load()

        # Create RobotManager and preload robots
        self.RM = RobotManager(cfg["robots_settings"])
        self.RM.preload_robot(self.world)

        # Add RobotManager to LunaryardController
        self.LC.add_robot_manager(self.RM)

        # Initialize the TCP Robot Manager with RobotManager reference
        self.robot_manager = TCP_RobotManager(
            self.RM.robots, self.physics_manager, self.RM
        )

        logger.info("TCP Lunaryard Manager initialized.")

    def handle_environment_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handles environment control commands (sun, terrain, rocks).

        Args:
            command: Command dictionary

        Returns:
            Response dictionary with success status
        """
        cmd_type = command.get("cmd")
        action = command.get("action")
        value = command.get("value")

        try:
            if cmd_type == "sun":
                return self._handle_sun_command(action, value, command)
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

    def _handle_sun_command(
        self, action: str, value: Any, command: Dict
    ) -> Dict[str, Any]:
        """Handle sun-related commands"""
        try:
            if action == "set_intensity":
                self.LC.set_sun_intensity(intensity=float(value))
                return {"success": True, "action": "set_intensity", "value": value}
            elif action == "set_color":
                self.LC.set_sun_color(color=tuple(value))
                return {"success": True, "action": "set_color", "value": value}
            elif action == "set_color_temperature":
                self.LC.set_sun_color_temperature(temperature=float(value))
                return {
                    "success": True,
                    "action": "set_color_temperature",
                    "value": value,
                }
            elif action == "set_angle":
                self.LC.set_sun_angle(angle=float(value))
                return {"success": True, "action": "set_angle", "value": value}
            elif action == "set_pose":
                position = command.get("position", [0, 0, 0])
                orientation = command.get("orientation", [1, 0, 0, 0])
                self.LC.set_sun_pose(
                    position=tuple(position), orientation=tuple(orientation)
                )
                return {"success": True, "action": "set_pose", "position": position}
            else:
                return {"success": False, "error": f"Unknown sun action: {action}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _handle_terrain_command(self, action: str, value: Any) -> Dict[str, Any]:
        """Handle terrain commands"""
        try:
            if action == "switch":
                self.LC.switch_terrain(flag=int(value))
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
                self.world.reset()
                return {"success": True, "action": "enable", "value": value}
            elif action == "randomize":
                num_rocks = int(value)
                self.LC.randomize_rocks(num=num_rocks)
                self.world.reset()
                return {"success": True, "action": "randomize", "value": num_rocks}
            else:
                return {"success": False, "error": f"Unknown rocks action: {action}"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def step(self, command_dict: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Called every simulation step.
        """
        if command_dict:
            for robot_name, cmd in command_dict.items():
                self.robot_manager.apply_command(robot_name, cmd)

        self.robot_manager.apply_persistent_commands()
        return self.robot_manager.get_state()
