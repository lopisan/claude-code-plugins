#!/bin/bash
# ROS PreToolUse Hook - Intercepts Bash commands and wraps ROS commands
# with a detected Docker wrapper script.
#
# Detection order:
# 1. ./run.sh with "# ros-wrapper" in first 5 lines
# 2. ./deploy/run.sh with "# ros-wrapper" in first 5 lines
# 3. CLAUDE.md with "ros-wrapper: <path>" directive
# 4. No wrapper - commands pass through unchanged

set -euo pipefail

# Read JSON input from stdin
input=$(cat)

# Extract the command from tool_input
command=$(echo "$input" | jq -r '.tool_input.command // empty')

# Skip if no command
[ -z "$command" ] && exit 0

# ROS command patterns to match
# Only wrap commands containing these ROS-specific tools
ROS_PATTERN='(^|[;&|]\s*)(rostopic|rosnode|rosservice|rosparam|roslaunch|rosbag|roswtf|rosrun|rospack|rosmsg|rossrv|roscore|catkin|catkin_make)(\s|$)'

# Skip if command doesn't contain ROS tools
if ! echo "$command" | grep -qE "$ROS_PATTERN"; then
    exit 0
fi

# Cache configuration
CACHE_DIR="${HOME}/.cache/claude-ros-plugin"
CACHE_FILE="${CACHE_DIR}/wrapper_path"

# Find .claude/ros_master_uri by walking up directory tree
find_master_file() {
    local dir="$PWD"
    while [ "$dir" != "/" ]; do
        if [ -f "$dir/.claude/ros_master_uri" ]; then
            echo "$dir/.claude/ros_master_uri"
            return 0
        fi
        dir=$(dirname "$dir")
    done
    echo ""
}

# Check if a file is a valid ros-wrapper
is_ros_wrapper() {
    local file="$1"
    [ -f "$file" ] && [ -x "$file" ] && head -5 "$file" | grep -q "# ros-wrapper"
}

# Detect wrapper script
detect_wrapper() {
    # Check ./run.sh
    if is_ros_wrapper "./run.sh"; then
        echo "./run.sh"
        return 0
    fi

    # Check ./deploy/run.sh
    if is_ros_wrapper "./deploy/run.sh"; then
        echo "./deploy/run.sh"
        return 0
    fi

    # Check CLAUDE.md for ros-wrapper directive
    if [ -f "CLAUDE.md" ]; then
        local wrapper_path
        wrapper_path=$(grep -m1 "^ros-wrapper:" "CLAUDE.md" 2>/dev/null | sed 's/^ros-wrapper:[[:space:]]*//' | tr -d '\r')
        if [ -n "$wrapper_path" ] && [ -f "$wrapper_path" ]; then
            echo "$wrapper_path"
            return 0
        fi
    fi

    # No wrapper found
    echo ""
    return 0
}

# Get cached wrapper path or detect it
get_wrapper() {
    mkdir -p "$CACHE_DIR"

    # Check if cache is valid (same working directory)
    if [ -f "$CACHE_FILE" ]; then
        local cached_dir cached_path
        cached_dir=$(head -1 "$CACHE_FILE")
        cached_path=$(tail -1 "$CACHE_FILE")

        if [ "$cached_dir" = "$(pwd)" ]; then
            echo "$cached_path"
            return 0
        fi
    fi

    # Detect and cache
    local wrapper
    wrapper=$(detect_wrapper)
    echo "$(pwd)" > "$CACHE_FILE"
    echo "$wrapper" >> "$CACHE_FILE"
    echo "$wrapper"
}

# Get wrapper
wrapper=$(get_wrapper)

# No wrapper? Pass through unchanged
[ -z "$wrapper" ] && exit 0

# Read master URI if available
master_file=$(find_master_file)
master_uri=""
[ -n "$master_file" ] && master_uri=$(cat "$master_file")

# Escape command for bash -c (escape double quotes and backslashes)
escaped_cmd=$(printf '%s' "$command" | sed 's/\\/\\\\/g; s/"/\\"/g')

# Build wrapped command
if [ -n "$master_uri" ]; then
    new_cmd="$wrapper -e ROS_MASTER_URI=$master_uri bash -c \"$escaped_cmd\""
else
    new_cmd="$wrapper bash -c \"$escaped_cmd\""
fi

# Output hook response with updated command
cat <<EOF
{
  "hookSpecificOutput": {
    "permissionDecision": "allow",
    "updatedInput": {"command": $(echo "$new_cmd" | jq -Rs .)}
  }
}
EOF
