---
description: Quick ROS system overview showing roscore status, active nodes, topics, and services
allowed-tools: Bash
model: haiku
---

Perform a comprehensive ROS system health check. Run these diagnostics:

## Step 1: Check ROS environment is sourced

```bash
if ! command -v roscore &> /dev/null; then
  echo "ERROR: ROS environment not sourced. Please run: source /opt/ros/<distro>/setup.bash"
  exit 1
fi
echo "ROS environment: OK"
```

## Step 2: Check roscore status

```bash
pgrep -a rosmaster 2>/dev/null && echo "ROSCORE_STATUS: RUNNING" || echo "ROSCORE_STATUS: NOT_RUNNING"
```

## Step 3: List active nodes (if roscore running)

```bash
timeout 3 rosnode list 2>&1
```

## Step 4: Count and list topics

```bash
timeout 3 rostopic list 2>&1
```

## Step 5: List available services

```bash
timeout 3 rosservice list 2>&1 | head -15
```

## Step 6: Check ROS environment variables

```bash
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH" | head -c 200
```

## Output Format

Provide a clear summary in this format:

### ROS System Status

**roscore:** [RUNNING / NOT RUNNING]

**Active Nodes:** [count]
- /node1
- /node2
...

**Active Topics:** [count]
- /topic1
- /topic2
...

**Services:** [count available]

**Environment:**
- ROS_DISTRO: noetic
- ROS_MASTER_URI: http://localhost:11311

### Recommendations
- If ROS not sourced: Tell user to source their ROS workspace
- If roscore not running: Suggest starting it with `roscore` or `roslaunch`
- If no nodes: Suggest launching nodes
- If issues detected: Suggest running `/diag` for diagnostics
