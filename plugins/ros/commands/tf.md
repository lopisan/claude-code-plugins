---
description: Inspect TF transform tree, lookup transforms between frames
argument-hint: [source_frame target_frame] | tree | frames | echo
allowed-tools: Bash
model: haiku
---

Inspect TF (Transform) coordinate frames. The user may provide frame names or subcommands in $ARGUMENTS.

## If no arguments or "frames" (list all frames):

```bash
timeout 3 rostopic echo -n 1 /tf 2>/dev/null | grep -o "child_frame_id: [^,]*" | sort -u || echo "No TF data available"
```

Or use tf_monitor:
```bash
timeout 3 rosrun tf tf_monitor 2>&1 | head -30
```

## If "tree" provided (show frame tree):

```bash
rosrun tf view_frames 2>&1
echo "Frame tree saved to frames.pdf in current directory"
```

Or get tree structure via tf_monitor:
```bash
timeout 5 rosrun tf tf_monitor 2>&1
```

## If two frame names provided (lookup transform):

```bash
# tf_echo <source_frame> <target_frame>
timeout 5 rosrun tf tf_echo <source_frame> <target_frame> 2>&1 | head -20
```

## If "echo" provided (raw TF topic):

```bash
# 5s timeout for data sampling
timeout 5 rostopic echo -n 5 /tf 2>&1
```

## If "static" provided (show static transforms):

```bash
# 5s timeout for data sampling
timeout 5 rostopic echo -n 5 /tf_static 2>&1
```

## Output Format

### For frame list:

Available TF frames:
- `base_link`
- `odom`
- `map`
- `camera_link`
- `lidar_link`

### For frame tree:

```
map
└── odom
    └── base_link
        ├── camera_link
        │   └── camera_optical_frame
        └── lidar_link
```

### For transform lookup:

**Transform:** base_link → odom

**Translation:**
- x: 1.234 m
- y: 0.567 m
- z: 0.000 m

**Rotation (quaternion):**
- x: 0.000
- y: 0.000
- z: 0.123
- w: 0.992

**Rotation (RPY):**
- roll: 0.000 rad (0.0°)
- pitch: 0.000 rad (0.0°)
- yaw: 0.247 rad (14.2°)

### Common Operations

- `/tf` - List available frames
- `/tf tree` - Generate frame tree visualization
- `/tf base_link odom` - Get transform between frames
- `/tf echo` - Show raw TF messages
- `/tf static` - Show static transforms

### Troubleshooting

If transforms are missing:
- Check if transform broadcaster is running
- Verify frame names match (case-sensitive)
- Use `rosrun tf tf_monitor` for timing diagnostics
- Use `rosrun rqt_tf_tree rqt_tf_tree` for visual debugging
