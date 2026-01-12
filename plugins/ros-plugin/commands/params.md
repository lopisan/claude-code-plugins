---
description: List, get, set, and manage ROS parameters
argument-hint: [param-name] | set <name> <value> | delete <name> | list [namespace]
allowed-tools: Bash
model: haiku
---

Manage ROS parameters. The user may provide various subcommands in $ARGUMENTS.

## If no arguments or "list" (list all parameters):

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam list 2>&1
```

## If "list <namespace>" provided:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam list <namespace> 2>&1
```

## If parameter name provided (starts with /):

### Get parameter value:
```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam get "$ARGUMENTS" 2>&1
```

## If "set <name> <value>" provided:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam set <name> <value> 2>&1
echo "Parameter set successfully"
```

## If "delete <name>" provided:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam delete <name> 2>&1
echo "Parameter deleted"
```

## If "dump [file]" provided:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam dump <file> 2>&1
```

## If "load <file>" provided:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
rosparam load <file> 2>&1
```

## Output Format

### For parameter list:

| Parameter | Namespace |
|-----------|-----------|
| /use_sim_time | / |
| /robot/wheel_radius | /robot |
| /robot/max_speed | /robot |

### For parameter value:

**Parameter:** /your_param
**Value:** `0.5`
**Type:** float

Or for complex values:
```yaml
wheel_radius: 0.1
max_speed: 1.0
joint_names:
  - joint1
  - joint2
```

### Common Operations

- `/params` - List all parameters
- `/params /use_sim_time` - Get specific parameter
- `/params set /rate 100` - Set parameter value
- `/params delete /old_param` - Delete parameter
- `/params list /robot` - List params in namespace
- `/params dump params.yaml` - Save all params to file
- `/params load params.yaml` - Load params from file
