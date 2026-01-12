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
/status
```

---

## Commands

### /status

Quick ROS system health check.

```
/status
```

**Output:**
- roscore status (running/stopped)
- Active nodes count and list
- Active topics count
- Available services

### /topics [topic-name]

Inspect ROS topics.

```
# List all topics with publishers/subscribers
/topics

# Inspect specific topic
/topics /sensor/data
```

**Output for specific topic:**
- Topic type and message structure
- Publishers and subscribers
- Publishing rate (Hz)
- Sample message content

### /services [service-name]

Manage ROS services.

```
# List all services
/services

# Inspect specific service
/services /node/get_loggers

# Call a service
/services call /set_bool "data: true"
```

### /params [param-name]

Manage ROS parameters.

```
# List all parameters
/params

# Get parameter value
/params /use_sim_time

# Set parameter
/params set /rate 100

# Delete parameter
/params delete /old_param
```

### /tf [source target]

Inspect TF transforms.

```
# List all frames
/tf

# Show frame tree
/tf tree

# Lookup transform
/tf base_link odom
```

### /build [package] [--clean]

Build catkin workspace.

```
# Build entire workspace
/build

# Build specific package
/build your_package

# Clean and rebuild
/build --clean
```

### /launch [package-or-file]

Find and inspect launch files.

```
# List all launch files in workspace
/launch

# Inspect package launch files
/launch your_package
```

**Output:**
- Launch file contents
- Nodes being launched
- Arguments and parameters
- Example launch command

### /record [start|stop|status|info]

Manage rosbag recording.

```
# Check status and list topics
/record

# Start recording (shows command)
/record start

# Stop active recording
/record stop

# Show bag file info
/record info my_recording.bag
```

### /diag

Run roswtf diagnostics.

```
/diag
```

**Output:**
- roswtf warnings and errors
- Environment configuration
- Node health checks
- Topic connection analysis

---

## Agents

Agents are triggered automatically based on conversation context.

### explorer (Blue)

**Trigger phrases:**
- "Explain how this ROS system works"
- "What nodes are running?"
- "How does data flow from sensor to output?"

**Capabilities:**
- Maps complete ROS graph
- Traces data flow paths
- Identifies node responsibilities

### debugger (Yellow)

**Trigger phrases:**
- "My node isn't receiving messages"
- "Topic not connecting"
- "Service call timing out"

**Capabilities:**
- Traces publisher-subscriber connections
- Verifies message type matching
- Checks namespace/remapping issues

### monitor (Cyan)

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

### message-types

Activated when discussing sensor_msgs, geometry_msgs, std_msgs, message fields and structures.

### launch-files

Activated when discussing launch file syntax, XML tags, conditionals, namespaces, remapping.

### troubleshooting

Activated when encountering errors: "Unable to communicate with master", "Topic not found", "Transform errors", build failures.

---

## Usage Examples

### Check System Status

```
User: /status

Claude: Checking ROS system status...

=== ROS System Status ===
roscore: RUNNING
Nodes: 5
Topics: 12
Services: 8
```

### Inspect a Topic

```
User: /topics /sensor/data

Claude: Inspecting topic /sensor/data...

Type: sensor_msgs/Type
Rate: 98.5 Hz
Publishers: /sensor_driver
Subscribers: /processor_node
```

### Debug Missing Data

```
User: My subscriber isn't receiving sensor data

Claude: [debugger agent activates]

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
│   ├── status.md             # /status
│   ├── topics.md             # /topics
│   ├── services.md           # /services
│   ├── params.md             # /params
│   ├── tf.md                 # /tf
│   ├── build.md              # /build
│   ├── launch.md             # /launch
│   ├── record.md             # /record
│   └── diag.md               # /diag
├── agents/
│   ├── explorer.md           # System exploration
│   ├── debugger.md           # Communication debugging
│   └── monitor.md            # Background monitoring
├── skills/
│   ├── message-types/
│   ├── launch-files/
│   └── troubleshooting/
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
