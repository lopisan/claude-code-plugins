---
command: config-diff
description: Compare current Claude Code config with a saved setup script
arguments: "<setup-script>"
user_invocable: true
---

# Config Diff Command

Compare your current Claude Code configuration with a previously saved setup script to see what has changed.

## Arguments

- `setup-script` (required): Path to a previously generated setup script (e.g., `claude_setup.sh`)

## Instructions

1. Verify the provided setup script exists and is readable.

2. Extract configuration from the setup script by parsing the commands:
   - `claude marketplace add <url/repo>` entries
   - `claude mcp add <name> ...` entries
   - `claude plugin install <id>` entries

3. Get current configuration from:
   - `~/.claude/plugins/known_marketplaces.json` for marketplaces
   - `~/.claude.json` for MCP servers
   - `~/.claude/settings.json` for enabled plugins

4. Compare and categorize items into:
   - **Added** (in current config but not in saved script)
   - **Removed** (in saved script but not in current config)
   - **Unchanged** (in both)

5. Display the diff in a clear format.

## Example Usage

```
/config-diff claude_setup.sh
/config-diff ~/backups/old_config.sh
```

## Example Output

```
Configuration Diff: claude_setup.sh vs Current Config

=== Marketplaces ===
  + phlubucek/new-plugins     (added)
  - old-marketplace/plugins   (removed)
  = phlubucek/claude-plugins  (unchanged)

=== MCP Servers ===
  + greptile                  (added)
  = context7                  (unchanged)
  = playwright                (unchanged)

=== Plugins ===
  + superpowers               (added)
  - old-plugin                (removed)
  = pr-review-toolkit         (unchanged)

Summary:
  Marketplaces: 1 added, 1 removed, 1 unchanged
  MCP Servers: 1 added, 0 removed, 2 unchanged
  Plugins: 1 added, 1 removed, 1 unchanged
```

## Parsing Logic

The command should use grep/regex to extract:

```bash
# Marketplaces
grep "claude marketplace add" script.sh | awk '{print $4}'

# MCP servers (name is the first arg after 'claude mcp add')
grep "claude mcp add" script.sh | awk '{print $4}'

# Plugins
grep "claude plugin install" script.sh | awk '{print $4}'
```

## Error Handling

- If the setup script doesn't exist, inform the user
- If the script is not a valid Claude setup script, show a warning
- If there are no differences, show "Configurations are identical"
