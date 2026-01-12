# ROS Plugin for Claude Code

Project-agnostic ROS1 plugin providing commands, agents, and skills for robotics development.

## Architecture

This plugin runs **inside a ROS container** where the ROS environment is available. All commands use direct Bash execution of ROS CLI tools (`rostopic`, `rosnode`, `rosservice`, etc.).

### Future Option: MCP Integration

The ROS MCP server provides 31 tools that could replace Bash commands for structured output and better error handling. This is documented as a future enhancement path when running outside the container or when MCP is available.

**MCP tools available (not currently used):**
- Topics: `ros_list_topics`, `ros_get_topic_info`, `ros_publish`, `ros_subscribe_once`, `ros_echo_topic`
- Services: `ros_list_services`, `ros_get_service_info`, `ros_call_service`
- Parameters: `ros_list_params`, `ros_get_param`, `ros_set_param`, `ros_delete_param`
- Nodes: `ros_list_nodes`, `ros_get_node_info`, `ros_ping_node`, `ros_get_master_info`
- TF: `ros_list_frames`, `ros_lookup_transform`, `ros_get_frame_tree`, `ros_can_transform`
- Messages: `ros_show_msg`, `ros_show_srv`, `ros_list_msg_types`

---

## Plugin Structure

```
ros-plugin/
├── .claude-plugin/
│   └── plugin.json           # Plugin manifest
├── commands/
│   ├── status.md             # System health check
│   ├── topics.md             # Topic inspection
│   ├── services.md           # Service management
│   ├── params.md             # Parameter management
│   ├── tf.md                 # TF transform inspection
│   ├── build.md              # Catkin workspace build
│   ├── launch.md             # Launch file management
│   ├── record.md             # Rosbag recording
│   └── diag.md               # System diagnostics
├── agents/
│   ├── explorer.md           # System architecture analysis
│   ├── debugger.md           # Communication debugging
│   └── monitor.md            # Runtime monitoring
├── skills/
│   ├── message-types/        # Message structure reference
│   ├── launch-files/         # Launch file guide
│   └── troubleshooting/      # Error diagnosis
├── hooks/
│   └── hooks.json            # Session start reminder
└── README.md
```

---

## Commands Reference

| Command | Description | Tools |
|---------|-------------|-------|
| `/status` | Quick system overview (nodes, topics, services) | Bash |
| `/topics [name]` | List/inspect topics with message samples | Bash |
| `/services [name]` | List/inspect/call services | Bash |
| `/params [name]` | List/get/set parameters | Bash |
| `/tf [frames]` | TF frame tree and transforms | Bash |
| `/build [pkg]` | Build catkin workspace | Bash |
| `/launch [pkg]` | Find and inspect launch files | Bash, Read, Glob |
| `/record [action]` | Manage rosbag recording | Bash |
| `/diag` | Run roswtf and system diagnostics | Bash |

---

## Agents Reference

| Agent | Trigger | Purpose |
|-------|---------|---------|
| `explorer` | "What nodes are running?", "Show system architecture" | Map ROS graph and data flow |
| `debugger` | "Topic not connecting", "Messages not received" | Debug communication issues |
| `monitor` | "Monitor topic rate", "Watch node health" | Continuous system monitoring |

---

## Environment Setup

All Bash commands require ROS environment. The session hook reminds to source:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${CATKIN_WS:-catkin_ws}/devel/setup.bash 2>/dev/null || true
```

---

## Implementation Notes

### Bash Commands Used

| ROS Tool | Used In |
|----------|---------|
| `roscore` | status (check if running) |
| `rosnode list/info/ping` | status, diag, agents |
| `rostopic list/info/type/echo/hz` | topics, agents |
| `rosservice list/info/type/call` | services |
| `rosparam list/get/set/delete` | params |
| `rosrun tf tf_echo/view_frames` | tf |
| `rosmsg show` | topics |
| `rossrv show` | services |
| `catkin build/catkin_make` | build |
| `roslaunch --files/--args` | launch |
| `rosbag record/info` | record |
| `roswtf` | diag |

### Timeout Policy

All ROS commands use consistent timeout values:
- **3s** - Quick operations (list nodes, topics, services)
- **5s** - Data sampling (rostopic echo, hz, TF lookups)

### Generic Examples

All examples use placeholders:
- Package: `your_package`
- Topic: `/sensor/data`, `/cmd_vel`, `/your_topic`
- Node: `/your_node`
- Launch: `robot.launch`
- Frame: `base_link`, `odom`, `map`
