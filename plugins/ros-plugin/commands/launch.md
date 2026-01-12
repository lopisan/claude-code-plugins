---
description: Find, inspect, and manage ROS launch files
argument-hint: [package-or-file] [args...]
allowed-tools: Bash, Read, Glob
---

Manage ROS launch files. Use $ARGUMENTS to specify a package name or launch file path.

## If no arguments provided - list all launch files:

```bash
echo "=== Launch files in workspace ==="
find ${CATKIN_WS:-catkin_ws}/src -name "*.launch" -type f 2>/dev/null | while read f; do
  pkg=$(basename $(dirname $(dirname $f)))
  echo "$pkg: $f"
done
```

```bash
echo "=== Installed ROS packages with launch files ==="
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
source ${CATKIN_WS:-catkin_ws}/devel/setup.bash 2>/dev/null || true
for pkg in $(rospack list-names 2>/dev/null | head -20); do
  pkg_path=$(rospack find $pkg 2>/dev/null)
  if [ -d "$pkg_path/launch" ]; then
    echo "$pkg: $pkg_path/launch/"
  fi
done | head -15
```

## If package name provided:

### Find package and its launch files:
```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
source ${CATKIN_WS:-catkin_ws}/devel/setup.bash 2>/dev/null || true

PKG_PATH=$(rospack find $ARGUMENTS 2>/dev/null)
if [ -n "$PKG_PATH" ]; then
  echo "Package: $ARGUMENTS"
  echo "Path: $PKG_PATH"
  echo ""
  echo "Launch files:"
  find $PKG_PATH -name "*.launch" -type f 2>/dev/null
else
  echo "Package '$ARGUMENTS' not found"
fi
```

### Read and parse the launch file:
Use the Read tool to read the launch file content, then parse:

1. **Arguments** (`<arg>` tags) - parameters that can be set at launch
2. **Nodes** (`<node>` tags) - what nodes will be started
3. **Parameters** (`<param>` tags) - what parameters are set
4. **Includes** (`<include>` tags) - other launch files included
5. **Remaps** (`<remap>` tags) - topic remappings

## If launch file path provided:

Read the file directly and parse its contents.

## Output Format

### Launch File Analysis

**Package:** [package_name]
**File:** [launch_file_path]

#### Arguments (configurable at launch)
| Argument | Default | Description |
|----------|---------|-------------|
| param1 | default_value | Description of parameter |
| param2 | 100 | Another parameter |

#### Nodes Started
| Node Name | Package | Type | Namespace |
|-----------|---------|------|-----------|
| your_node | your_package | node_type | / |

#### Parameters Set
| Parameter | Value |
|-----------|-------|
| ~param1 | $(arg param1) |
| ~param2 | $(arg param2) |

#### Included Launch Files
- [other.launch]

### How to Launch

```bash
# Basic launch
roslaunch [package] [file.launch]

# With arguments
roslaunch [package] [file.launch] arg1:=value1 arg2:=value2

# Example for this launch file:
roslaunch your_package robot.launch param1:=value1 param2:=100
```

### Tips
- Use `roslaunch --ros-args [package] [file]` to see all arguments
- Add `--screen` to see node output in terminal
- Use `rosnode list` after launch to verify nodes started
