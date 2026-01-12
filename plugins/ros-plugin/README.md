# ROS Plugin for Claude Code

A project-agnostic Claude Code plugin for ROS1 (Noetic) development with commands, agents, and skills for robotics workflows.

## Installation

### Option 1: Development Testing

Use the `--plugin-dir` flag to load the plugin during development:

```bash
claude --plugin-dir ./ros-plugin
```

This loads the plugin directly without installation. Useful for testing changes.

### Option 2: Project Installation (Shared with Team)

Add to your project's `.claude/settings.json`:

```json
{
  "plugins": {
    "ros-plugin": {
      "source": "./ros-plugin"
    }
  }
}
```

This shares the plugin with all collaborators via version control.

### Option 3: User-Global Installation

Install for personal use across all projects:

```bash
cp -r ros-plugin ~/.claude/plugins/
```

### Verify Installation

After loading or installing the plugin:
```
/ros:status
```

---

## Commands

### /ros:status

Quick ROS system health check.

```
/ros:status
```

**Output:**
- roscore status (running/stopped)
- Active nodes count and list
- Active topics count
- Available services

### /ros:topics [topic-name]

Inspect ROS topics.

```
# List all topics with publishers/subscribers
/ros:topics

# Inspect specific topic
/ros:topics /sensor/data
```

**Output for specific topic:**
- Topic type and message structure
- Publishers and subscribers
- Publishing rate (Hz)
- Sample message content

### /ros:services [service-name]

Manage ROS services.

```
# List all services
/ros:services

# Inspect specific service
/ros:services /node/get_loggers

# Call a service
/ros:services call /set_bool "data: true"
```

### /ros:params [param-name]

Manage ROS parameters.

```
# List all parameters
/ros:params

# Get parameter value
/ros:params /use_sim_time

# Set parameter
/ros:params set /rate 100

# Delete parameter
/ros:params delete /old_param
```

### /ros:tf [source target]

Inspect TF transforms.

```
# List all frames
/ros:tf

# Show frame tree
/ros:tf tree

# Lookup transform
/ros:tf base_link odom
```

### /ros:build [package] [--clean]

Build catkin workspace.

```
# Build entire workspace
/ros:build

# Build specific package
/ros:build your_package

# Clean and rebuild
/ros:build --clean
```

### /ros:launch [package-or-file]

Find and inspect launch files.

```
# List all launch files in workspace
/ros:launch

# Inspect package launch files
/ros:launch your_package
```

**Output:**
- Launch file contents
- Nodes being launched
- Arguments and parameters
- Example launch command

### /ros:record [start|stop|status|info]

Manage rosbag recording.

```
# Check status and list topics
/ros:record

# Start recording (shows command)
/ros:record start

# Stop active recording
/ros:record stop

# Show bag file info
/ros:record info my_recording.bag
```

### /ros:diag

Run roswtf diagnostics.

```
/ros:diag
```

**Output:**
- roswtf warnings and errors
- Environment configuration
- Node health checks
- Topic connection analysis

---

## Agents

Agents are triggered automatically based on conversation context.

### ros-explorer (Blue)

**Trigger phrases:**
- "Explain how this ROS system works"
- "What nodes are running?"
- "How does data flow from sensor to output?"

**Capabilities:**
- Maps complete ROS graph
- Traces data flow paths
- Identifies node responsibilities

### ros-debugger (Yellow)

**Trigger phrases:**
- "My node isn't receiving messages"
- "Topic not connecting"
- "Service call timing out"

**Capabilities:**
- Traces publisher-subscriber connections
- Verifies message type matching
- Checks namespace/remapping issues

### ros-monitor (Cyan)

**Trigger phrases:**
- "Monitor the sensor topic rate"
- "Watch for dropped messages"
- "Keep an eye on node health"

**Capabilities:**
- Continuous rate monitoring
- Node health pinging
- Anomaly detection

---

## Skills

Skills provide contextual guidance when relevant topics are discussed.

### ros-message-types

Activated when discussing sensor_msgs, geometry_msgs, std_msgs, message fields and structures.

### ros-launch-files

Activated when discussing launch file syntax, XML tags, conditionals, namespaces, remapping.

### ros-troubleshooting

Activated when encountering errors: "Unable to communicate with master", "Topic not found", "Transform errors", build failures.

---

## Usage Examples

### Check System Status

```
User: /ros:status

Claude: Checking ROS system status...

=== ROS System Status ===
roscore: RUNNING
Nodes: 5
Topics: 12
Services: 8
```

### Inspect a Topic

```
User: /ros:topics /sensor/data

Claude: Inspecting topic /sensor/data...

Type: sensor_msgs/Type
Rate: 98.5 Hz
Publishers: /sensor_driver
Subscribers: /processor_node
```

### Debug Missing Data

```
User: My subscriber isn't receiving sensor data

Claude: [ros-debugger agent activates]

Let me trace the connection...

Publisher: /sensor_driver publishes to /sensor/data
Your subscriber expects: /robot/sensor/data

ISSUE: Topic name mismatch!

Fix: Remap in launch file or change subscriber topic name
```

---

## File Structure

```
ros-plugin/
├── .claude-plugin/
│   └── plugin.json           # Plugin manifest
├── commands/
│   ├── ros:status.md         # /ros:status
│   ├── ros:topics.md         # /ros:topics
│   ├── ros:services.md       # /ros:services
│   ├── ros:params.md         # /ros:params
│   ├── ros:tf.md             # /ros:tf
│   ├── ros:build.md          # /ros:build
│   ├── ros:launch.md         # /ros:launch
│   ├── ros:record.md         # /ros:record
│   └── ros:diag.md           # /ros:diag
├── agents/
│   ├── ros-explorer.md       # System exploration
│   ├── ros-debugger.md       # Communication debugging
│   └── ros-monitor.md        # Background monitoring
├── skills/
│   ├── ros-message-types/
│   ├── ros-launch-files/
│   └── ros-troubleshooting/
├── hooks/
│   └── hooks.json            # Session hooks
├── CLAUDE.md                 # Development notes
└── README.md                 # This file
```

---

## Environment

Set `CATKIN_WS` environment variable to specify your catkin workspace path (defaults to `catkin_ws`).

```bash
export CATKIN_WS=/path/to/your/catkin_ws
```

Commands automatically source:
```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
source ${CATKIN_WS:-catkin_ws}/devel/setup.bash
```

---

## Requirements

- ROS1 installed (Noetic, Melodic, etc.) at `/opt/ros/<distro>/`
- Catkin workspace (configurable via `CATKIN_WS`)
- Claude Code CLI
