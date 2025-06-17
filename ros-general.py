import asyncio
import json
import os
import select
import subprocess
import sys
import time
from typing import Any, Optional

import websockets
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("ros-general")

# WebSocket server configuration
WEBSOCKET_SERVER_HOST = "localhost"
WEBSOCKET_SERVER_PORT = 8765


def get_ros_env():
    """Get ROS environment with proper variable inheritance."""
    env = os.environ.copy()

    # Ensure ROS_DOMAIN_ID is set
    if "ROS_DOMAIN_ID" not in env:
        env["ROS_DOMAIN_ID"] = "37"  # Default value

    # Ensure other important ROS variables
    if "ROS_DISTRO" not in env:
        env["ROS_DISTRO"] = "humble"


def run_ros_command(command: list[str], timeout: Optional[float] = None) -> str:
    """Helper function to run ROS2 commands with proper error handling and environment.

    Note: ROS 2 environment should be sourced when starting the MCP server.
    """
    try:
        env = get_ros_env()

        if timeout:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                env=env,  # Pass environment explicitly
            )
        else:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                env=env,  # Pass environment explicitly
            )

        if result.returncode != 0:
            return f"Error: {result.stderr.strip()}"

        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return f"Command timed out after {timeout} seconds"
    except subprocess.CalledProcessError as e:
        return f"Error: {e}"
    except Exception as e:
        return f"Unexpected error: {e}"


def run_ros_command_with_bash(command: str, timeout: Optional[float] = None) -> str:
    """Helper function for complex bash commands that need shell processing.

    Note: ROS 2 environment should be sourced when starting the MCP server.
    """
    try:
        env = get_ros_env()

        # Create a comprehensive bash command that sources ROS and sets environment
        full_command = f"""
        source /opt/ros/humble/setup.bash
        if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi
        export ROS_DOMAIN_ID={env.get('ROS_DOMAIN_ID', '37')}
        export ROS_DISTRO={env.get('ROS_DISTRO', 'humble')}
        {command}
        """.strip()

        if timeout:
            result = subprocess.run(
                ["bash", "-c", full_command],
                capture_output=True,
                text=True,
                timeout=timeout,
                env=env,
            )
        else:
            result = subprocess.run(
                ["bash", "-c", full_command], capture_output=True, text=True, env=env
            )

        if result.returncode != 0:
            return f"Error: {result.stderr.strip()}"

        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return f"Command timed out after {timeout} seconds"
    except subprocess.CalledProcessError as e:
        return f"Error: {e}"
    except Exception as e:
        return f"Unexpected error: {e}"


async def send_gui_command(command: str, args: dict = None) -> str:
    """Send GUI command to WebSocket server running on local environment.

    Args:
        command: Command to execute ('launch_rqt_graph', 'launch_rviz', etc.)
        args: Additional arguments for the command

    Returns:
        Response from the WebSocket server
    """
    try:
        uri = f"ws://{WEBSOCKET_SERVER_HOST}:{WEBSOCKET_SERVER_PORT}"

        message = {"command": command, "args": args or {}}

        async with websockets.connect(uri) as websocket:
            await websocket.send(json.dumps(message))
            response = await websocket.recv()
            return json.loads(response).get("result", "No response from server")

    except ConnectionRefusedError:
        return f"Error: Could not connect to GUI server at {uri}. Make sure the WebSocket server is running."
    except asyncio.TimeoutError:
        return "Error: Connection to GUI server timed out."
    except Exception as e:
        return f"Error communicating with GUI server: {e}"


@mcp.tool()
async def list_topics() -> str:
    """Displays a list of currently accessible ROS2 topics.

    Example: Used when the user asks "What topics are available?"
    """
    return run_ros_command(["ros2", "topic", "list"])


@mcp.tool()
async def check_topic_status(topic_name: str) -> str:
    """Checks whether a specific ROS2 topic is actively publishing messages.

    Args:
        topic_name: The name of the ROS2 topic to check

    Example: Used when the user asks "Is the topic /scan publishing data?"
    """
    if not topic_name:
        return "Please specify a topic name (e.g., /scan)"

    try:
        env = get_ros_env()

        # Start ros2 topic echo with proper environment
        process = subprocess.Popen(
            ["ros2", "topic", "echo", topic_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )

        # Wait up to 3 seconds for output from stdout
        ready_fds, _, _ = select.select([process.stdout], [], [], 3.0)

        if not ready_fds:
            process.kill()
            return f"The topic '{topic_name}' is recognized, but no data has been published within 3 seconds."

        # Read a line from stdout
        line = process.stdout.readline()
        process.kill()

        # Check output content
        if (
            "does not appear to be published yet" in line
            or "Could not determine the type" in line
        ):
            return f"The topic '{topic_name}' is not currently being published or has an unknown type.\n\nDetails:\n{line.strip()}"
        elif line.strip():
            return f"The topic '{topic_name}' appears to be active and publishing data:\n{line.strip()}"
        else:
            return f"The topic '{topic_name}' is recognized, but no data has been published yet."

    except Exception as e:
        return f"An error occurred while checking the topic: {e}"


@mcp.tool()
async def find_ros2_package(pkg_name: str) -> str:
    """Searches for available ROS 2 packages that match a given keyword.

    Args:
        pkg_name: Partial or full name of the package to search for

    Example: "Are there any packages related to turtle?" → ros2 pkg list | grep turtle
    """
    command = f"ros2 pkg list | grep {pkg_name}"
    result = run_ros_command_with_bash(command)
    return result or f"No package found matching '{pkg_name}'."


@mcp.tool()
async def run_ros2_executable(package: str, executable: str) -> str:
    """Runs a ROS 2 executable from a specified package.

    Args:
        package: Name of the ROS2 package
        executable: Name of the executable to run

    Example: "Start the turtlesim node" → ros2 run turtlesim turtlesim_node
    """
    try:
        env = get_ros_env()
        process = subprocess.Popen(["ros2", "run", package, executable], env=env)
        return f"Started executable '{executable}' from package '{package}' with PID {process.pid}."
    except Exception as e:
        return f"Failed to run executable: {e}"


@mcp.tool()
async def list_ros2_nodes() -> str:
    """Lists currently running ROS 2 nodes.

    Example: "Which nodes are currently running?" → ros2 node list
    """
    result = run_ros_command(["ros2", "node", "list"])
    return result or "No nodes are currently running."


@mcp.tool()
async def get_ros2_node_info(node_name: str) -> str:
    """Shows detailed information about a specific ROS 2 node.

    Args:
        node_name: Name of the node (e.g., /turtlesim)

    Example: "Tell me about node /turtlesim" → ros2 node info /turtlesim
    """
    result = run_ros_command(["ros2", "node", "info", node_name])
    if "Error:" in result:
        return f"Node '{node_name}' not found or not running."
    return result


@mcp.tool()
async def publish_ros2_topic(
    topic: str, msg_type: str, msg_content: str, duration: int
) -> str:
    """Publishes a message to a specific ROS 2 topic for a given duration (in seconds).

    Args:
        topic: Topic name to publish to (e.g., "/chatter")
        msg_type: ROS 2 message type (e.g., "std_msgs/msg/String")
        msg_content: Message to send (e.g., "data: Hello")
        duration: Duration in seconds to continue publishing

    Example:
        topic: "/chatter"
        msg_type: "std_msgs/msg/String"
        msg_content: "data: Hello"
        duration: 3
    """
    try:
        env = get_ros_env()
        process = subprocess.Popen(
            ["ros2", "topic", "pub", topic, msg_type, msg_content],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )
        time.sleep(duration)
        process.terminate()
        return f"Published to '{topic}' for {duration} seconds."
    except Exception as e:
        return f"Error publishing to topic: {e}"


@mcp.tool()
async def check_ros2_topic_hz(topic_name: str) -> str:
    """Displays the publishing frequency (Hz) of a given ROS 2 topic.

    Args:
        topic_name: Name of the topic (e.g., /scan)

    Example: "Check the message rate on /scan" → ros2 topic hz /scan
    """
    try:
        env = get_ros_env()
        full_command = f"""
        source /opt/ros/humble/setup.bash
        if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi
        export ROS_DOMAIN_ID={env.get('ROS_DOMAIN_ID', '37')}
        export ROS_DISTRO={env.get('ROS_DISTRO', 'humble')}
        ros2 topic hz {topic_name}
        """.strip()
        process = subprocess.Popen(
            ["bash", "-c", full_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=env,
        )

        # Wait a few seconds for output to accumulate
        time.sleep(5)
        process.terminate()
        stdout, _ = process.communicate()

        # Extract lines containing useful stats
        lines = stdout.strip().splitlines()
        stats_lines = [
            line
            for line in lines
            if line.startswith("average rate") or line.strip().startswith("min:")
        ]

        return (
            "\n".join(stats_lines)
            if stats_lines
            else "No frequency data was received. The topic may be inactive."
        )
    except Exception as e:
        return f"Failed to check topic hz: {e}"


@mcp.tool()
async def list_ros2_services() -> str:
    """Lists available ROS 2 services.

    Example: No arguments needed. Just returns the list.
    """
    return run_ros_command(["ros2", "service", "list"])


@mcp.tool()
async def call_ros2_service(service_name: str, srv_type: str, request: str) -> str:
    """Calls a ROS 2 service with the specified name, type, and request.

    Args:
        service_name: Name of the service to call
        srv_type: Type of the service (e.g., std_srvs/srv/Empty)
        request: Request payload in string format (e.g., "{}")

    Example:
        service_name: "/clear_costmap"
        srv_type: "std_srvs/srv/Empty"
        request: "{}"
    """
    return run_ros_command(["ros2", "service", "call", service_name, srv_type, request])


@mcp.tool()
async def list_ros2_actions() -> str:
    """Lists available ROS 2 actions.

    Example: No arguments needed. Just returns the list.
    """
    return run_ros_command(["ros2", "action", "list"])


@mcp.tool()
async def send_ros2_action_goal(action_name: str, action_type: str, goal: str) -> str:
    """Sends a goal to a ROS 2 action.

    Args:
        action_name: Name of the action
        action_type: Action type (e.g., example_interfaces/action/Fibonacci)
        goal: Goal message (e.g., "{order: 5}")

    Example:
        action_name: "/fibonacci"
        action_type: "example_interfaces/action/Fibonacci"
        goal: "{order: 5}"
    """
    return run_ros_command(
        ["ros2", "action", "send_goal", action_name, action_type, goal]
    )


@mcp.tool()
async def run_ros2_doctor() -> str:
    """Runs ros2 doctor to check ROS 2 environment setup and issues.

    Example: No arguments needed. Just runs the doctor tool.
    """
    return run_ros_command(["ros2", "doctor"])


@mcp.tool()
async def show_ros2_interface(msg_type: str) -> str:
    """Shows the interface (definition) for a given ROS 2 message or service type.

    Args:
        msg_type: The interface type to show (e.g., std_msgs/msg/String)

    Example: msg_type: "std_msgs/msg/String"
    """
    return run_ros_command(["ros2", "interface", "show", msg_type])


@mcp.tool()
async def launch_rqt_graph() -> str:
    """Launch rqt_graph GUI tool via WebSocket server.

    Example: Launch rqt_graph to visualize ROS2 node connections
    """
    return await send_gui_command("launch_rqt_graph")


@mcp.tool()
async def launch_rviz() -> str:
    """Launch RViz2 GUI tool via WebSocket server.

    Example: Launch RViz2 for visualization
    """
    return await send_gui_command("launch_rviz")


@mcp.tool()
async def launch_turtlesim() -> str:
    """Launch turtlesim GUI application via WebSocket server.

    Example: Launch turtlesim for turtle simulation
    """
    return await send_gui_command("launch_turtlesim")


@mcp.tool()
async def launch_gazebo() -> str:
    """Launch Gazebo simulation environment via WebSocket server.

    Example: Launch Gazebo for physics simulation
    """
    return await send_gui_command("launch_gazebo")


@mcp.tool()
async def launch_turtlebot3_world() -> str:
    """Launch TurtleBot3 world in Gazebo via WebSocket server.

    Example: Launch TurtleBot3 in a world environment for simulation
    """
    return await send_gui_command("launch_turtlebot3_world")


@mcp.tool()
async def launch_turtlebot3_empty_world() -> str:
    """Launch TurtleBot3 in empty world in Gazebo via WebSocket server.

    Example: Launch TurtleBot3 in an empty world for simulation
    """
    return await send_gui_command("launch_turtlebot3_empty_world")


@mcp.tool()
async def get_topic_info(topic_name: str) -> str:
    """Get detailed information about a specific ROS2 topic.

    Args:
        topic_name: Name of the topic to get info about

    Example: Get information about /cmd_vel topic
    """
    return run_ros_command(["ros2", "topic", "info", topic_name])


@mcp.tool()
async def debug_ros2_environment() -> str:
    """Debug ROS 2 environment variables and setup.

    Example: Check current ROS 2 environment configuration
    """
    env = get_ros_env()
    debug_command = f"""
    echo "=== ROS 2 Environment Debug ==="
    echo "ROS_DISTRO: {env.get('ROS_DISTRO', 'Not set')}"
    echo "ROS_DOMAIN_ID: {env.get('ROS_DOMAIN_ID', 'Not set')}"
    echo "DISPLAY: {env.get('DISPLAY', 'Not set')}"
    echo "=== Available topics ==="
    ros2 topic list
    echo "=== Available nodes ==="
    ros2 node list
    """.strip()

    return run_ros_command_with_bash(debug_command)


@mcp.tool()
async def echo_ros2_topic(topic_name: str, count: int = 1) -> str:
    """Echo messages from a ROS2 topic for a specified number of messages.

    Args:
        topic_name: Name of the topic to echo
        count: Number of messages to capture (default: 1)

    Example: Echo 5 messages from /scan topic
    """
    if count == 1:
        return run_ros_command(
            ["ros2", "topic", "echo", topic_name, "--once"], timeout=10
        )
    else:
        return run_ros_command(
            ["ros2", "topic", "echo", topic_name, f"--times", str(count)], timeout=10
        )


if __name__ == "__main__":
    mcp.run(transport="stdio")
