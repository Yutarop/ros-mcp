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
To enable node communication between the MCP server and the local ROS environment, both must be configured with the same `ROS_DOMAIN_ID` on the same local network.

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
To use the MCP server correctly with Claude Desktop, you need to modify the claude_desktop_config.json file with appropriate values.
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
##### **💡 Replace the following placeholders:**
- `/ABSOLUTE/PATH/TO/PARENT/FOLDER/ros-mcp`
Replace with the absolute path to the directory ros-mcp. 
For example: /home/ubuntu/ros-mcp

- `YOUR_ROS_DOMAIN_ID`
  Replace with the ROS domain ID used in your environment to enable ROS communication between the MCP server and your local machine. If you're not sure what your current domain ID is, you can check it by running the following command in your terminal:
  ```bash
  echo $ROS_DOMAIN_ID
  ```
  If nothing is printed, it means the domain ID is not explicitly set and the default value 0 will be used.
  Alternatively, you can set it manually using the following command:
  ```bash
  export ROS_DOMAIN_ID=10  # Replace 10 with your desired domain ID
  ```
- `/ABSOLUTE/PATH/TO/PARENT/FOLDER/ros-general.py`
Replace with the absolute path to your ros-general.py script.
For example: /home/ubuntu/ros-mcp/ros-general.py

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
- [ ] Let the MCP server detect whether a ROS package contains nodes that require a GUI, and launch the GUI if necessary. Currently, this detection is hard-coded for specific packages.
