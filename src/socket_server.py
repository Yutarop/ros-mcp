#!/usr/bin/env python3
"""
WebSocket server for launching ROS2 GUI applications from MCP server.
This server runs on the local environment and can launch GUI applications
that require X11 display access.
"""

import asyncio
import json
import logging
import os
import signal
import subprocess
import sys
from typing import Any, Dict

import websockets

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class GUILauncher:
    """Class to handle GUI application launches."""

    def __init__(self):
        self.running_processes: Dict[str, subprocess.Popen] = {}
        self.setup_environment()

    def setup_environment(self):
        """Setup ROS2 environment variables."""
        self.env = os.environ.copy()

        # Set default values if not already set
        if "ROS_DOMAIN_ID" not in self.env:
            self.env["ROS_DOMAIN_ID"] = "0"

        if "DISPLAY" not in self.env:
            self.env["DISPLAY"] = ":0"

        if "TURTLEBOT3_MODEL" not in self.env:
            self.env["TURTLEBOT3_MODEL"] = "burger"

        # Source ROS2 setup
        setup_command = f"""
        export ROS_DOMAIN_ID={self.env['ROS_DOMAIN_ID']}
        export DISPLAY={self.env['DISPLAY']}
        export TURTLEBOT3_MODEL={self.env['TURTLEBOT3_MODEL']}
        """
        self.env["ROS_SETUP"] = setup_command.strip()

        logger.info(
            f"Environment setup - ROS_DISTRO: {self.env.get('ROS_DISTRO')}, "
            f"ROS_DOMAIN_ID: {self.env.get('ROS_DOMAIN_ID')}, "
            f"DISPLAY: {self.env.get('DISPLAY')}, "
            f"TURTLEBOT3_MODEL: {self.env.get('TURTLEBOT3_MODEL')}"
        )

    async def launch_rqt_graph(self) -> str:
        """Launch rqt_graph GUI application."""
        try:
            # Kill existing rqt_graph processes
            await self._kill_existing_process("rqt_graph")

            # Launch rqt_graph
            command = f"""
            {self.env['ROS_SETUP']}
            ros2 run rqt_graph rqt_graph
            """

            process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.env,
                preexec_fn=os.setsid,  # Create new process group
            )

            self.running_processes["rqt_graph"] = process

            # Wait a moment to check if the process started successfully
            await asyncio.sleep(2)

            if process.poll() is None:
                logger.info(f"rqt_graph launched successfully with PID {process.pid}")
                return f"rqt_graph launched successfully with PID {process.pid}"
            else:
                stdout, stderr = process.communicate()
                error_msg = (
                    f"rqt_graph failed to start. Error: {stderr.decode().strip()}"
                )
                logger.error(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Error launching rqt_graph: {str(e)}"
            logger.error(error_msg)
            return error_msg

    async def launch_rviz(self) -> str:
        """Launch RViz2 GUI application."""
        try:
            # Kill existing rviz processes
            await self._kill_existing_process("rviz2")

            # Launch rviz2
            command = f"""
            {self.env['ROS_SETUP']}
            ros2 run rviz2 rviz2
            """

            process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.env,
                preexec_fn=os.setsid,  # Create new process group
            )

            self.running_processes["rviz2"] = process

            # Wait a moment to check if the process started successfully
            await asyncio.sleep(2)

            if process.poll() is None:
                logger.info(f"RViz2 launched successfully with PID {process.pid}")
                return f"RViz2 launched successfully with PID {process.pid}"
            else:
                stdout, stderr = process.communicate()
                error_msg = f"RViz2 failed to start. Error: {stderr.decode().strip()}"
                logger.error(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Error launching RViz2: {str(e)}"
            logger.error(error_msg)
            return error_msg

    async def launch_turtlesim(self) -> str:
        """Launch turtlesim GUI application."""
        try:
            # Kill existing turtlesim processes
            await self._kill_existing_process("turtlesim_node")

            # Launch turtlesim
            command = f"""
            {self.env['ROS_SETUP']}
            ros2 run turtlesim turtlesim_node
            """

            process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.env,
                preexec_fn=os.setsid,  # Create new process group
            )

            self.running_processes["turtlesim_node"] = process

            # Wait a moment to check if the process started successfully
            await asyncio.sleep(2)

            if process.poll() is None:
                logger.info(f"turtlesim launched successfully with PID {process.pid}")
                return f"turtlesim launched successfully with PID {process.pid}"
            else:
                stdout, stderr = process.communicate()
                error_msg = (
                    f"turtlesim failed to start. Error: {stderr.decode().strip()}"
                )
                logger.error(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Error launching turtlesim: {str(e)}"
            logger.error(error_msg)
            return error_msg

    async def launch_gazebo(self) -> str:
        """Launch Gazebo simulation environment."""
        try:
            # Kill existing gazebo processes
            await self._kill_existing_process("gazebo")

            # Launch gazebo
            command = f"""
            {self.env['ROS_SETUP']}
            gazebo
            """

            process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.env,
                preexec_fn=os.setsid,  # Create new process group
            )

            self.running_processes["gazebo"] = process

            # Wait a moment to check if the process started successfully
            await asyncio.sleep(3)  # Gazebo takes longer to start

            if process.poll() is None:
                logger.info(f"Gazebo launched successfully with PID {process.pid}")
                return f"Gazebo launched successfully with PID {process.pid}"
            else:
                stdout, stderr = process.communicate()
                error_msg = f"Gazebo failed to start. Error: {stderr.decode().strip()}"
                logger.error(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Error launching Gazebo: {str(e)}"
            logger.error(error_msg)
            return error_msg

    async def launch_turtlebot3_world(self) -> str:
        """Launch TurtleBot3 world in Gazebo."""
        try:
            # Kill existing turtlebot3/gazebo processes
            await self._kill_existing_process("gazebo")
            await self._kill_existing_process("robot_state_publisher")
            await self._kill_existing_process("spawn_entity")

            # Launch TurtleBot3 world
            command = f"""
            {self.env['ROS_SETUP']}
            ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
            """

            process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.env,
                preexec_fn=os.setsid,  # Create new process group
            )

            self.running_processes["turtlebot3_world"] = process

            # Wait longer for TurtleBot3 world to start
            await asyncio.sleep(5)

            if process.poll() is None:
                logger.info(
                    f"TurtleBot3 world launched successfully with PID {process.pid}"
                )
                return f"TurtleBot3 world launched successfully with PID {process.pid}"
            else:
                stdout, stderr = process.communicate()
                error_msg = f"TurtleBot3 world failed to start. Error: {stderr.decode().strip()}"
                logger.error(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Error launching TurtleBot3 world: {str(e)}"
            logger.error(error_msg)
            return error_msg

    async def launch_turtlebot3_empty_world(self) -> str:
        """Launch TurtleBot3 in empty world in Gazebo."""
        try:
            # Kill existing turtlebot3/gazebo processes
            await self._kill_existing_process("gazebo")
            await self._kill_existing_process("robot_state_publisher")
            await self._kill_existing_process("spawn_entity")

            # Launch TurtleBot3 empty world
            command = f"""
            {self.env['ROS_SETUP']}
            ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
            """

            process = subprocess.Popen(
                ["bash", "-c", command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=self.env,
                preexec_fn=os.setsid,  # Create new process group
            )

            self.running_processes["turtlebot3_empty_world"] = process

            # Wait longer for TurtleBot3 empty world to start
            await asyncio.sleep(5)

            if process.poll() is None:
                logger.info(
                    f"TurtleBot3 empty world launched successfully with PID {process.pid}"
                )
                return f"TurtleBot3 empty world launched successfully with PID {process.pid}"
            else:
                stdout, stderr = process.communicate()
                error_msg = f"TurtleBot3 empty world failed to start. Error: {stderr.decode().strip()}"
                logger.error(error_msg)
                return error_msg

        except Exception as e:
            error_msg = f"Error launching TurtleBot3 empty world: {str(e)}"
            logger.error(error_msg)
            return error_msg

    async def _kill_existing_process(self, process_name: str):
        """Kill existing processes by name."""
        try:
            # Kill by process name
            subprocess.run(
                ["pkill", "-f", process_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            # Also kill if we have it tracked
            if process_name in self.running_processes:
                process = self.running_processes[process_name]
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                del self.running_processes[process_name]

            await asyncio.sleep(1)  # Give time for processes to terminate

        except Exception as e:
            logger.warning(
                f"Warning: Could not kill existing {process_name} processes: {e}"
            )

    async def handle_command(self, command: str, args: Dict[str, Any]) -> str:
        """Handle incoming commands."""
        logger.info(f"Received command: {command} with args: {args}")

        if command == "launch_rqt_graph":
            return await self.launch_rqt_graph()
        elif command == "launch_rviz":
            return await self.launch_rviz()
        elif command == "launch_turtlesim":
            return await self.launch_turtlesim()
        elif command == "launch_gazebo":
            return await self.launch_gazebo()
        elif command == "launch_turtlebot3_world":
            return await self.launch_turtlebot3_world()
        elif command == "launch_turtlebot3_empty_world":
            return await self.launch_turtlebot3_empty_world()
        else:
            return f"Unknown command: {command}"

    def cleanup(self):
        """Clean up running processes."""
        for name, process in self.running_processes.items():
            try:
                if process.poll() is None:
                    logger.info(f"Terminating {name} (PID: {process.pid})")
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception as e:
                logger.warning(f"Error terminating {name}: {e}")


class GUIWebSocketServer:
    """WebSocket server for handling GUI launch requests."""

    def __init__(self, host="localhost", port=8765):
        self.host = host
        self.port = port
        self.launcher = GUILauncher()

    async def handle_client(self, websocket):
        """Handle client connections."""
        client_addr = websocket.remote_address
        logger.info(f"Client connected from {client_addr}")

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    command = data.get("command")
                    args = data.get("args", {})

                    if not command:
                        response = {"error": "No command specified"}
                    else:
                        result = await self.launcher.handle_command(command, args)
                        response = {"result": result}

                    await websocket.send(json.dumps(response))

                except json.JSONDecodeError:
                    error_response = {"error": "Invalid JSON format"}
                    await websocket.send(json.dumps(error_response))
                except Exception as e:
                    error_response = {"error": f"Error processing command: {str(e)}"}
                    await websocket.send(json.dumps(error_response))

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client {client_addr} disconnected")
        except Exception as e:
            logger.error(f"Error handling client {client_addr}: {e}")

    async def start_server(self):
        """Start the WebSocket server."""
        logger.info(f"Starting GUI WebSocket server on {self.host}:{self.port}")

        server = await websockets.serve(self.handle_client, self.host, self.port)

        logger.info(f"GUI WebSocket server started on ws://{self.host}:{self.port}")
        return server

    def cleanup(self):
        """Cleanup resources."""
        self.launcher.cleanup()


async def main():
    """Main function to run the WebSocket server."""
    server_instance = GUIWebSocketServer()

    def signal_handler(signum, frame):
        logger.info("Received signal to shutdown...")
        server_instance.cleanup()
        sys.exit(0)

    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        server = await server_instance.start_server()

        # Keep the server running
        await server.wait_closed()

    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        server_instance.cleanup()
        logger.info("Server shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)
