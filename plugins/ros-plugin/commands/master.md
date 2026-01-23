---
description: Discover and select ROS master for this session. Run before other ROS commands.
allowed-tools: Bash, AskUserQuestion
model: haiku
---

# ROS Master Discovery

Discover available ROS masters and select one for this session.

## Steps

1. **Check localhost first** (runs on host, not in container):
   ```bash
   bash "${CLAUDE_PLUGIN_ROOT}/scripts/find_ros_master.sh"
   ```

2. **Parse JSON result**: `{"masters": ["127.0.0.1"]}` or `{"masters": []}`

3. **If localhost found**: Auto-select and confirm
   ```
   ROS master found at localhost
   URI: http://127.0.0.1:11311
   ```

4. **If localhost not found**: Ask user for remote hosts
   Use AskUserQuestion: "No ROS master on localhost. Enter remote host IP/hostname to check:"
   Then run:
   ```bash
   bash "${CLAUDE_PLUGIN_ROOT}/scripts/find_ros_master.sh" <user-provided-host>
   ```

5. **Store the selection**:
   Remember the selected ROS_MASTER_URI for subsequent ROS commands this session.
   When running ROS commands, prepend: `ROS_MASTER_URI=http://<selected>:11311`

## Output

```
ROS Master: http://127.0.0.1:11311
Ready for ROS commands.
```

Or if user provided host:
```
ROS Master: http://192.168.1.50:11311
Ready for ROS commands.
```
