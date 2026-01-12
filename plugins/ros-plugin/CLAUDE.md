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
│   ├── ros:status.md         # System health check
│   ├── ros:topics.md         # Topic inspection
│   ├── ros:services.md       # Service management
│   ├── ros:params.md         # Parameter management
│   ├── ros:tf.md             # TF transform inspection
│   ├── ros:build.md          # Catkin workspace build
│   ├── ros:launch.md         # Launch file management
│   ├── ros:record.md         # Rosbag recording
│   └── ros:diag.md           # System diagnostics
├── agents/
│   ├── ros-explorer.md       # System architecture analysis
│   ├── ros-debugger.md       # Communication debugging
│   └── ros-monitor.md        # Runtime monitoring
├── skills/
│   ├── ros-message-types/    # Message structure reference
│   ├── ros-launch-files/     # Launch file guide
│   └── ros-troubleshooting/  # Error diagnosis
├── hooks/
│   └── hooks.json            # Session start reminder
└── README.md
```

---

## Commands Reference

| Command | Description | Tools |
|---------|-------------|-------|
| `/ros:status` | Quick system overview (nodes, topics, services) | Bash |
| `/ros:topics [name]` | List/inspect topics with message samples | Bash |
| `/ros:services [name]` | List/inspect/call services | Bash |
| `/ros:params [name]` | List/get/set parameters | Bash |
| `/ros:tf [frames]` | TF frame tree and transforms | Bash |
| `/ros:build [pkg]` | Build catkin workspace | Bash |
| `/ros:launch [pkg]` | Find and inspect launch files | Bash, Read, Glob |
| `/ros:record [action]` | Manage rosbag recording | Bash |
| `/ros:diag` | Run roswtf and system diagnostics | Bash |

---

## Agents Reference

| Agent | Trigger | Purpose |
|-------|---------|---------|
| `ros-explorer` | "What nodes are running?", "Show system architecture" | Map ROS graph and data flow |
| `ros-debugger` | "Topic not connecting", "Messages not received" | Debug communication issues |
| `ros-monitor` | "Monitor topic rate", "Watch node health" | Continuous system monitoring |

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
| `roscore` | ros:status (check if running) |
| `rosnode list/info/ping` | ros:status, ros:diag, agents |
| `rostopic list/info/type/echo/hz` | ros:topics, agents |
| `rosservice list/info/type/call` | ros:services |
| `rosparam list/get/set/delete` | ros:params |
| `rosrun tf tf_echo/view_frames` | ros:tf |
| `rosmsg show` | ros:topics |
| `rossrv show` | ros:services |
| `catkin build/catkin_make` | ros:build |
| `roslaunch --files/--args` | ros:launch |
| `rosbag record/info` | ros:record |
| `roswtf` | ros:diag |

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
