---
description: Discover and select ROS master for this session. Run before other ROS commands.
allowed-tools: Bash, AskUserQuestion
model: haiku
---

# ROS Master Discovery

Discover available ROS masters and let the user select one for this session.

## Steps

1. **Scan for ROS masters** (runs on host, not in container):
   ```bash
   bash "${CLAUDE_PLUGIN_ROOT}/scripts/find_ros_master.sh"
   ```

2. **Parse JSON result**:
   - `{"masters": [...], "external": [...], "local": "..."}`

3. **Handle results based on count**:

   **If no masters found**:
   - Use AskUserQuestion to ask for a specific IP/hostname to check
   - Run script again with that host

   **If exactly one master found**:
   - Auto-select and confirm the single option
   - Display: `ROS Master: http://<ip>:11311`

   **If multiple masters found (2+)**:
   - Use AskUserQuestion with dropdown options showing all discovered IPs
   - Include labels: "(local)" for the local IP, "(external)" for others
   - Example options: "192.168.0.121 (local)", "192.168.0.240 (external)"

4. **Store the selection**:
   Remember the selected ROS_MASTER_URI for subsequent ROS commands this session.
   When running ROS commands, prepend: `ROS_MASTER_URI=http://<selected>:11311`

## Output

Single master:
```
ROS Master: http://127.0.0.1:11311
Ready for ROS commands.
```

Multiple masters (after user selection):
```
ROS Master: http://192.168.0.240:11311
Ready for ROS commands.
```
