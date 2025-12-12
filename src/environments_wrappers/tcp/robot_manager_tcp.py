import logging
from typing import Dict, Any, Tuple
import numpy as np
from src.robots.robot import RobotManager, Robot
from src.physics.physics_scene import PhysicsSceneManager

logger = logging.getLogger(__name__)


class TCP_RobotManager:
    def __init__(
        self,
        robots: Dict[str, Robot],
        physics_manager: PhysicsSceneManager,
        robot_manager: RobotManager = None,
    ) -> None:
        self.robots = robots
        self.physics_manager = physics_manager
        self.robot_manager = robot_manager  # Reference to RobotManager for spawn/reset
        self.last_cmd_vel = {}  # Store last command per robot
        self.use_wheel_physics = True  # Default to wheel physics (realistic motion)

        logger.info(
            f"TCP Robot Manager initialized with {len(self.robots)} robots (wheel physics: {self.use_wheel_physics})."
        )

    def set_wheel_physics_mode(self, enabled: bool) -> None:
        """
        Toggle between wheel-based physics and root-based driving.

        Args:
            enabled: True for realistic wheel physics, False for simple root driving
        """
        self.use_wheel_physics = enabled
        mode = "wheel physics" if enabled else "root driving"
        logger.info(f"Switched to {mode} mode")

    def handle_robot_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Handles robot management commands (spawn, teleport, reset).

        Args:
            command: Command dictionary

        Returns:
            Response dictionary with success status
        """
        cmd_type = command.get("cmd")

        try:
            if cmd_type == "spawn_robot":
                return self.spawn_robot(
                    robot_name=command.get("robot_name"),
                    usd_path=command.get("usd_path"),
                    position=command.get("position", [0, 0, 0.5]),
                    orientation=command.get("orientation", [1, 0, 0, 0]),
                )
            elif cmd_type == "teleport":
                return self.teleport_robot(
                    robot_name=command.get("robot_name"),
                    position=command.get("position"),
                    orientation=command.get("orientation"),
                )
            elif cmd_type == "reset_robot":
                return self.reset_robot(command.get("robot_name"))
            elif cmd_type == "reset_all_robots":
                return self.reset_all_robots()
            else:
                return {"success": False, "error": f"Unknown robot command: {cmd_type}"}
        except Exception as e:
            logger.error(f"Error handling robot command {cmd_type}: {e}")
            return {"success": False, "error": str(e)}

    def spawn_robot(
        self,
        robot_name: str,
        usd_path: str,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> Dict[str, Any]:
        """Spawn a new robot dynamically"""
        if not self.robot_manager:
            return {"success": False, "error": "RobotManager not available"}

        try:
            self.robot_manager.add_robot(
                usd_path=usd_path,
                robot_name=robot_name,
                p=position,
                q=orientation,
            )
            # Update local robots reference
            self.robots = self.robot_manager.robots
            logger.info(f"Spawned robot {robot_name} at position {position}")
            return {"success": True, "robot_name": robot_name}
        except Exception as e:
            logger.error(f"Failed to spawn robot {robot_name}: {e}")
            return {"success": False, "error": str(e)}

    def teleport_robot(
        self,
        robot_name: str,
        position: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float],
    ) -> Dict[str, Any]:
        """Teleport robot to a specific position and orientation"""
        if not self.robot_manager:
            return {"success": False, "error": "RobotManager not available"}

        try:
            self.robot_manager.teleport_robot(
                robot_name=robot_name,
                position=np.array(position),
                orientation=np.array(orientation),
            )
            logger.info(f"Teleported {robot_name} to position {position}")
            return {"success": True, "robot_name": robot_name, "position": position}
        except Exception as e:
            logger.error(f"Failed to teleport robot {robot_name}: {e}")
            return {"success": False, "error": str(e)}

    def reset_robot(self, robot_name: str) -> Dict[str, Any]:
        """Reset a specific robot to its initial state"""
        if not self.robot_manager:
            return {"success": False, "error": "RobotManager not available"}

        try:
            self.robot_manager.reset_robot(robot_name=robot_name)
            # Clear velocity command for this robot
            if robot_name in self.last_cmd_vel:
                del self.last_cmd_vel[robot_name]
            logger.info(f"Reset robot {robot_name}")
            return {"success": True, "robot_name": robot_name}
        except Exception as e:
            logger.error(f"Failed to reset robot {robot_name}: {e}")
            return {"success": False, "error": str(e)}

    def reset_all_robots(self) -> Dict[str, Any]:
        """Reset all robots to their initial states"""
        if not self.robot_manager:
            return {"success": False, "error": "RobotManager not available"}

        try:
            self.robot_manager.reset_robots()
            # Clear all velocity commands
            self.last_cmd_vel.clear()
            logger.info("Reset all robots")
            return {"success": True, "robot_count": len(self.robots)}
        except Exception as e:
            logger.error(f"Failed to reset all robots: {e}")
            return {"success": False, "error": str(e)}

    def apply_command(self, robot_name: str, command: Dict[str, Any]) -> None:
        """
        Applies a command received via TCP to the specified robot.
        Expected command format: {"cmd": "vel", "val": [vx, wz]}
        Commands persist until changed - no need to continuously send.
        Setting velocity to [0, 0] clears the persistent command.
        """
        if robot_name not in self.robots:
            logger.warning(f"Robot {robot_name} not found. Ignoring command.")
            return

        cmd_type = command.get("cmd")
        if cmd_type == "vel":
            values = command.get("val", [0.0, 0.0])
            if len(values) == 2:
                linear_x = float(values[0])
                angular_z = float(values[1])

                # If both velocities are zero, clear the persistent command (robot stops)
                if abs(linear_x) < 0.001 and abs(angular_z) < 0.001:
                    if robot_name in self.last_cmd_vel:
                        del self.last_cmd_vel[robot_name]
                        logger.info(
                            f"Cleared velocity command for {robot_name} (stopped)"
                        )
                else:
                    # Store the command - it will be persistently applied each frame
                    self.last_cmd_vel[robot_name] = (linear_x, angular_z)
                    logger.debug(
                        f"Updated velocity for {robot_name}: linear={linear_x}, angular={angular_z}"
                    )
            else:
                logger.warning(f"Invalid velocity command values: {values}")
        elif cmd_type == "wheel_mode":
            # Special command to toggle physics mode
            self.set_wheel_physics_mode(bool(command.get("val", False)))
        else:
            logger.warning(f"Unknown command type: {cmd_type}")

    def apply_persistent_commands(self) -> None:
        """
        Applies the last received velocity command to all robots.
        Called every simulation frame to maintain persistent motion.
        """
        for robot_name, (linear_x, angular_z) in self.last_cmd_vel.items():
            if robot_name in self.robots:
                robot = self.robots[robot_name]

                if self.use_wheel_physics:
                    # Use wheel-based physics (wheels will rotate)
                    # Check if robot has wheel joints configured
                    if (
                        hasattr(robot, "_wheel_joint_names")
                        and robot._wheel_joint_names
                    ):
                        # Proper differential drive logic handles all cases (straight, turn, combined)
                        # We use a heuristic for the wheelbase scaling since we don't have the exact robot parameters
                        # This allows for combined forward/backward motion AND rotation
                        rotation_component = (
                            angular_z * 0.5
                        )  # 0.5 approximates (wheelbase / 2)

                        left_speed = linear_x - rotation_component
                        right_speed = linear_x + rotation_component

                        robot._set_wheels_velocity(left_speed, "left")
                        robot._set_wheels_velocity(right_speed, "right")

                    else:
                        logger.warning(
                            f"Robot {robot_name} has no wheel joints configured. Falling back to root drive."
                        )
                        robot.drive_root(linear_x, angular_z)
                else:
                    # Use root-based driving (no wheel rotation, but reliable)
                    robot.drive_root(linear_x, angular_z)

    def get_state(self) -> Dict[str, Any]:
        """
        Returns the current state of all robots (position, orientation, velocity).
        """
        state = {}
        for name, robot in self.robots.items():
            try:
                position, orientation = robot.get_pose()
                pos_list = (
                    [position.x, position.y, position.z]
                    if hasattr(position, "x")
                    else list(position)
                )
                quat_list = (
                    [orientation.x, orientation.y, orientation.z, orientation.w]
                    if hasattr(orientation, "w")
                    else list(orientation)
                )

                state[name] = {
                    "position": pos_list,
                    "orientation": quat_list,
                    "linear_velocity": self.last_cmd_vel.get(name, [0.0, 0.0]),
                    "wheel_physics_mode": self.use_wheel_physics,
                }
            except Exception as e:
                logger.error(f"Error getting state for {name}: {e}")
        return state
