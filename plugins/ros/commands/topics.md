---
description: List and inspect ROS topics with types, publishers, subscribers, and rates
argument-hint: [topic-name]
allowed-tools: Bash
---

Inspect ROS topics in the system. The user may optionally provide a topic name as $ARGUMENTS.

## If no topic specified (list all topics):

```bash
rostopic list -v 2>&1
```

This shows all topics with their publishers and subscribers count.

## If topic name provided ($ARGUMENTS contains a topic):

Run these commands to get comprehensive topic information:

### Get topic info (publishers/subscribers):
```bash
rostopic info "$ARGUMENTS" 2>&1
```

### Get message type:
```bash
rostopic type "$ARGUMENTS" 2>&1
```

### Show message structure:
```bash
TOPIC_TYPE=$(rostopic type "$ARGUMENTS" 2>/dev/null)
if [ -n "$TOPIC_TYPE" ]; then
  rosmsg show "$TOPIC_TYPE" 2>&1
fi
```

### Measure publishing rate (5s timeout for rate sampling):
```bash
timeout 5 rostopic hz "$ARGUMENTS" 2>&1 || echo "No messages received or topic not publishing"
```

### Echo latest message (5s timeout for data sampling):
```bash
timeout 5 rostopic echo -n 1 "$ARGUMENTS" 2>&1 || echo "No message received within timeout"
```

## Output Format

### For topic list:
Show a formatted table of topics with their type and publisher/subscriber counts.

### For specific topic:

**Topic:** /topic/name
**Type:** sensor_msgs/Type
**Publishers:** [count] nodes
**Subscribers:** [count] nodes
**Rate:** X.X Hz (measured)

**Message Structure:**
```
[rosmsg show output]
```

**Latest Message:**
```
[rostopic echo output]
```

### Common Operations
Suggest relevant follow-up commands like:
- `rostopic echo /topic` - continuous monitoring
- `rostopic pub /topic type data` - publish test message
- `rosbag record /topic` - record topic data
