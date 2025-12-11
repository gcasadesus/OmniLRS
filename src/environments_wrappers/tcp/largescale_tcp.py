import logging
from typing import Dict, Any

from src.environments.large_scale_lunar import LargeScaleController
from src.environments_wrappers.tcp.robot_manager_tcp import TCP_RobotManager
from src.physics.physics_scene import PhysicsSceneManager
from src.robots.robot import RobotManager
from isaacsim.core.api.world import World

logger = logging.getLogger(__name__)


class TCP_LargeScaleManager:
    def __init__(
        self, cfg: Dict[str, Any], physics_manager: PhysicsSceneManager, world: World
    ) -> None:
        self.physics_manager = physics_manager
        self.world = world

        # Initialize the Environment Controller
        self.LC = LargeScaleController(**cfg)
        self.LC.load()

        # Create RobotManager and preload robots
        self.RM = RobotManager(cfg["robots_settings"])
        self.RM.preload_robot(self.world)

        # Add RobotManager to LargeScaleController
        self.LC.add_robot_manager(self.RM)

        # Initialize the TCP Robot Manager with RobotManager reference
        self.robot_manager = TCP_RobotManager(
            self.RM.robots, self.physics_manager, self.RM
        )

        logger.info("TCP Large Scale Manager initialized.")

    def handle_environment_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handles environment control commands (sun).

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
                self.LC.set_sun_color(color=list(value))
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

    def step(self, command_dict: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Called every simulation step.
        """
        if command_dict:
            for robot_name, cmd in command_dict.items():
                self.robot_manager.apply_command(robot_name, cmd)

        self.robot_manager.apply_persistent_commands()
        return self.robot_manager.get_state()
