## ROS MCP
**ROS MCP** is a MCP server designed for controlling robots in ROS environments using natural language. 
It supports communication via ROS topics, services, and actions, and works with any ROS message type.

## Demo
## Componets
(fig)
- **Socket Server** ([`socket_server.py`](https://github.com/Yutarop/ros-mcp/blob/main/src/socket_server.py)): A lightweight server that runs on the local machine to handle GUI-related operations. 
It receives instructions from the MCP server to launch tools such as `Gazebo` or `rqt_graph` on the local display.

- **MCP Server** ([`ros-general.py`](https://github.com/Yutarop/ros-mcp/blob/main/ros-general.py)): A Python-based server that implements the MCP.
It processes natural language input, maps it to ROS commands, and communicates with the socket server.
To enable inter-node communication between the MCP server and the local ROS environment, both must be configured with the same `ROS_DOMAIN_ID` on the same local network.

## Getting Started
#### Requirements
- ROS 2 Humble Hawksbill (This project has only been tested with ROS 2 Humble. Compatibility with other ROS 2 distributions is not guaranteed.)
- Python 3.10+
- uv package manager
- Other dependencies as listed in `pyproject.toml`

#### Git clone
```bash
$ git clone git@github.com:Yutarop/ros-mcp.git
$ cd ros-mcp
```

#### Activate .venv and install dependencies
```bash
$ uv venv
$ source .venv/bin/activate
$ uv pip install -e .
```

#### Claude Settings (`claude_desktop_config.json`)
```
{
  "mcpServers": {
    "ros-general": {
      "command": "uv",
      "args": [
        "--directory",
        "/ABSOLUTE/PATH/TO/PARENT/FOLDER/ros-mcp",
        "run",
        "bash",
        "-c",
        "export ROS_LOG_DIR=/tmp && export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-YOUR_ROS_DOMAIN_ID} && source /opt/ros/humble/setup.bash && python3 /ABSOLUTE/PATH/TO/PARENT/FOLDER/ros-general.py"
      ]
    }
  }
}
```
#### Cursor Settings 

## Run
#### Start MCP server
```bash
(ros-mcp) $ uv run ros-general.py
```
#### Start socket server
```bash
$ python3 socket_server.py
```
 > 💡not in the virtual environment!

## Provided Tools
## Upcomming
- [ ] Add tools for creating and controlling objects in Gazebo
- [ ] Convert [TurtleBot3 agent](https://github.com/Yutarop/turtlebot3_agent) tools into MCP-compatible tools using LangChain MCP adapters
- [ ] Let the MCP server detect whether a ROS package contains nodes that require a GUI, and launch the GUI if necessary.
