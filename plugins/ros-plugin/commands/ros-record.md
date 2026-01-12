---
description: Manage rosbag recording - start, stop, and inspect bag files
argument-hint: [start|stop|status|info] [topics-or-bagfile]
allowed-tools: Bash
---

Manage rosbag recording sessions. Commands via $ARGUMENTS:
- `start` or `start /topic1 /topic2` - Start recording
- `stop` - Stop active recording
- `status` - Check recording status
- `info <bagfile>` - Show bag file info
- (no args) - Show status and available topics

## Check current recording status:

```bash
echo "=== Recording Status ==="
if pgrep -a "rosbag record" >/dev/null 2>&1; then
  echo "STATUS: RECORDING"
  pgrep -a "rosbag record"
  echo ""
  echo "Active bag files:"
  ls -la *.bag.active 2>/dev/null || echo "No active bag files in current directory"
else
  echo "STATUS: NOT_RECORDING"
fi
```

## List available topics for recording:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
echo "=== Available Topics ==="
rostopic list 2>/dev/null | while read topic; do
  hz=$(timeout 3 rostopic hz "$topic" 2>&1 | grep "average rate" | awk '{print $3}' || echo "?")
  type=$(rostopic type $topic 2>/dev/null)
  printf "%-40s %10s Hz  %s\n" "$topic" "$hz" "$type"
done
```

## Start recording:

### Record all topics:
```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
echo "Starting rosbag record -a..."
echo "Command: rosbag record -a -O recording_$(date +%Y%m%d_%H%M%S).bag"
# Note: Actually starting requires user confirmation or background execution
```

### Record specific topics:
```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
TOPICS="$SPECIFIED_TOPICS"  # e.g., "/sensor/data /camera/image"
echo "Starting rosbag record for: $TOPICS"
echo "Command: rosbag record $TOPICS -O recording_$(date +%Y%m%d_%H%M%S).bag"
```

## Stop recording:

```bash
echo "Stopping rosbag recording..."
pkill -INT rosbag 2>&1
sleep 1
if pgrep "rosbag record" >/dev/null 2>&1; then
  echo "Recording still active, sending SIGTERM..."
  pkill -TERM rosbag
else
  echo "Recording stopped successfully"
fi

echo ""
echo "Completed bag files:"
ls -lh *.bag 2>/dev/null | tail -5
```

## Show bag file info:

```bash
source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash
BAGFILE="$SPECIFIED_BAGFILE"
if [ -f "$BAGFILE" ]; then
  rosbag info $BAGFILE 2>&1
else
  echo "Bag file not found: $BAGFILE"
  echo ""
  echo "Available bag files:"
  ls -lh *.bag 2>/dev/null || echo "No bag files in current directory"
fi
```

## Output Format

### Recording Status

**Status:** RECORDING / NOT_RECORDING
**Active Recording:** [filename.bag.active]
**Duration:** [if available]

### Available Topics for Recording

| Topic | Rate | Type | Size Est. |
|-------|------|------|-----------|
| /sensor/data | 100 Hz | sensor_msgs/Type | ~50 KB/s |
| /camera/image | 30 Hz | sensor_msgs/Image | ~30 MB/s |

### Recording Commands

```bash
# Record all topics (caution: high disk usage)
rosbag record -a -O my_recording.bag

# Record specific topics (recommended)
rosbag record /topic1 /topic2 -O my_recording.bag

# Record with compression
rosbag record /topic1 /topic2 --lz4 -O my_recording.bag

# Record with duration limit (5 minutes)
rosbag record /topic1 --duration=300 -O my_recording.bag
```

### Playback Commands
```bash
# Play bag file
rosbag play recording.bag

# Play at half speed
rosbag play -r 0.5 recording.bag

# Loop playback
rosbag play -l recording.bag
```
