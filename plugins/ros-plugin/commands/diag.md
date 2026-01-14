---
description: Run roswtf diagnostics and comprehensive ROS system health analysis
allowed-tools: Bash
---

Run comprehensive ROS diagnostics to identify system issues.

## Step 1: Run roswtf

```bash
roswtf 2>&1
```

## Step 2: Check ROS environment variables

```bash
echo "=== ROS Environment ==="
echo "ROS_DISTRO: ${ROS_DISTRO:-NOT_SET}"
echo "ROS_MASTER_URI: ${ROS_MASTER_URI:-NOT_SET}"
echo "ROS_IP: ${ROS_IP:-NOT_SET}"
echo "ROS_HOSTNAME: ${ROS_HOSTNAME:-NOT_SET}"
echo "ROS_PACKAGE_PATH: ${ROS_PACKAGE_PATH:-NOT_SET}"
echo "PYTHONPATH (ROS): $(echo $PYTHONPATH | grep -o '/opt/ros[^:]*' | head -1)"
```

## Step 3: Check roscore/master connectivity

```bash
echo "=== Master Connectivity ==="
timeout 3 rostopic list >/dev/null 2>&1 && echo "Master: REACHABLE" || echo "Master: UNREACHABLE"
```

## Step 4: Check for dead/unresponsive nodes

```bash
echo "=== Node Health Check ==="
for node in $(rosnode list 2>/dev/null); do
  if timeout 2 rosnode ping -c 1 $node 2>&1 | grep -q "reply"; then
    echo "$node: OK"
  else
    echo "$node: UNRESPONSIVE"
  fi
done
```

## Step 5: Check topic publisher/subscriber connectivity

```bash
echo "=== Topic Connectivity ==="
for topic in $(rostopic list 2>/dev/null | head -10); do
  info=$(rostopic info $topic 2>/dev/null)
  pubs=$(echo "$info" | grep -c "Publishers:")
  subs=$(echo "$info" | grep -c "Subscribers:")
  if echo "$info" | grep -A1 "Publishers:" | grep -q "None"; then
    echo "$topic: NO_PUBLISHERS"
  elif echo "$info" | grep -A1 "Subscribers:" | grep -q "None"; then
    echo "$topic: NO_SUBSCRIBERS (may be expected)"
  else
    echo "$topic: CONNECTED"
  fi
done
```

## Step 6: Check network configuration

```bash
echo "=== Network Configuration ==="
echo "Hostname: $(hostname)"
echo "IP addresses: $(hostname -I | tr ' ' '\n' | head -3 | tr '\n' ' ')"
echo "ROS_MASTER_URI points to: ${ROS_MASTER_URI:-http://localhost:11311}"
```

## Output Format

### ROS Diagnostics Report

**Overall Status:** [HEALTHY / WARNINGS / ERRORS]

#### roswtf Results
[Summary of roswtf findings]

#### Environment
| Variable | Value | Status |
|----------|-------|--------|
| ROS_DISTRO | noetic | OK |
| ROS_MASTER_URI | http://... | OK/ISSUE |

#### Node Health
| Node | Status |
|------|--------|
| /node1 | OK |
| /node2 | UNRESPONSIVE |

#### Topic Connectivity
| Topic | Status |
|-------|--------|
| /topic1 | CONNECTED |
| /topic2 | NO_PUBLISHERS |

### Issues Found
1. [Issue description and fix]
2. [Issue description and fix]

### Recommendations
- [Specific fix recommendations based on findings]
