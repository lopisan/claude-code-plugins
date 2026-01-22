---
command: config-save
description: Export Claude Code configuration to a portable setup script
arguments: "[options] [output_file]"
user_invocable: true
---

# Config Save Command

Export your current Claude Code configuration (marketplaces, MCP servers, plugins) to a portable setup script that can restore the same configuration on a different machine.

## Arguments

- `output-file` (optional): Path for the generated script. Defaults to `claude_setup.sh` in the current directory.

## Options

- `--dry-run` - Show generated script without writing to file
- `--no-cleanup` - Generate setup without cleanup phase (won't remove existing config)
- `--mcp-only` - Only export MCP servers
- `--plugins-only` - Only export plugins
- `--marketplaces-only` - Only export marketplaces

## Instructions

1. Parse the user's arguments to determine:
   - The output file path (if provided)
   - Any flags (`--dry-run`, `--no-cleanup`, selective export flags)

2. Check if the required config files exist and count the items:
   - `~/.claude/plugins/known_marketplaces.json` for marketplaces
   - `~/.claude.json` for MCP servers
   - `~/.claude/settings.json` for enabled plugins

3. Run the generation script located at `${CLAUDE_PLUGIN_ROOT}/generate_claude_setup.sh` with the appropriate arguments and flags.

4. After generation, display a summary showing:
   - The output file path (or "dry-run mode" if using `--dry-run`)
   - Count of marketplaces captured
   - Count of MCP servers captured
   - Count of plugins captured
   - Instructions for restoring on another machine (unless dry-run)

## Example Usage

```
/config-save
/config-save my_config.sh
/config-save --dry-run
/config-save --no-cleanup backup.sh
/config-save --mcp-only mcp_config.sh
/config-save --plugins-only --marketplaces-only
```

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

## Dry-Run Output

```
=== Generated Script (dry-run) ===

#!/bin/bash
# Claude Code setup script - Marketplaces, Plugins, and MCP Servers
# Generated on: Thu Jan 22 10:30:00 UTC 2026
...

=== End of Script ===

Captured:
  Marketplaces: 2
  MCP Servers: 5
  Plugins: 3
```

## Error Handling

- If no config files exist, inform the user that there's nothing to export
- If `jq` is not installed, show an error with installation instructions
- If the script fails, show the error message to the user
