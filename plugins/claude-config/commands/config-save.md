---
command: config-save
description: Export Claude Code configuration to a portable setup script
arguments: "[output-file]"
user_invocable: true
---

# Config Save Command

Export your current Claude Code configuration (marketplaces, MCP servers, plugins) to a portable setup script that can restore the same configuration on a different machine.

## Arguments

- `output-file` (optional): Path for the generated script. Defaults to `claude_setup.sh` in the current directory.

## Instructions

1. Determine the output file path:
   - If the user provided an argument, use it as the output path
   - Otherwise, use `claude_setup.sh` in the current working directory

2. Check if the required config files exist and count the items:
   - `~/.claude/plugins/known_marketplaces.json` for marketplaces
   - `~/.claude.json` for MCP servers
   - `~/.claude/settings.json` for enabled plugins

3. Run the generation script located at `{plugin_dir}/generate_claude_setup.sh` with the output path as an argument.

4. After generation, display a summary showing:
   - The output file path
   - Count of marketplaces captured
   - Count of MCP servers captured
   - Count of plugins captured
   - Instructions for restoring on another machine

## Example Output Format

```
Configuration saved to: ./claude_setup.sh

Captured:
  Marketplaces: 2
  MCP Servers: 5
  Plugins: 3

To restore on another machine:
  chmod +x claude_setup.sh
  ./claude_setup.sh
```

## Error Handling

- If no config files exist, inform the user that there's nothing to export
- If the script fails, show the error message to the user
